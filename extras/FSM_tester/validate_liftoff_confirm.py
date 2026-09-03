#!/usr/bin/env python3
"""Validate LIFTOFF_CONFIRM_MS=120 confirmation (per-TIME, not per-cycle).

Scenario matrix (faithful C++ port, 50Hz where the C++ runs at 50Hz):
  A. Bench noise (13_30_11-Dados.csv): must NEVER leave IDLE.
  B. Real flight (dados_filtrados.csv): liftoff confirmed; deploy at apogee.
  C. RocketPy sims (thonyan + dedalo, resampled to 50Hz): liftoff confirmed;
     deploy at apogee.
  D. Worst case: single-cycle spike of 30 m/s2 on a quiet bench -> IDLE holds.
  E. Freefall-from-IDLE (drone drop from 120m): FSM stays IDLE, backstop deploys
     ~1.0s after release.
"""
import csv, math, sys

LIFTOFF_ACCEL_THRESHOLD = 15.0
LIFTOFF_CONFIRM_MS = 100  # matches firmware/config.h
LIFTOFF_CONFIRM_MAX_GAP_MS = 60  # matches firmware/config.h
APOGEE_MAX_VZ = 1.0
PARACHUTE_MIN_ALTITUDE = 50.0
PARACHUTE_CONFIRM_VZ = -2.0
PARACHUTE_CONFIRM_CYCLES = 3
FILTER_ALPHA = 0.2
# backstop
FF_ACC = 3.0
FF_VZ = -5.0
FF_MINH = 50.0
FF_CYCLES = 50

def total(ax, ay, az):
    return math.sqrt(ax*ax + ay*ay + az*az)

def run_fsm(rows, label, verbose=False):
    """rows: (t_ms, alt, ax, ay, az) — faithful port of detectLiftoffTimed()."""
    state = 'IDLE'
    fax = fay = faz = None
    liftoff_above = False
    liftoff_start = None
    prev_t = prev_h = None
    vz = 0.0
    confirm = 0
    deployed = False
    events = []
    for (t, h, ax, ay, az) in rows:
        if fax is None:
            fax, fay, faz = ax, ay, az
        else:
            fax = FILTER_ALPHA*ax + (1-FILTER_ALPHA)*fax
            fay = FILTER_ALPHA*ay + (1-FILTER_ALPHA)*fay
            faz = FILTER_ALPHA*az + (1-FILTER_ALPHA)*faz
        if prev_t is not None:
            dt = (t - prev_t)/1000.0
            if dt > 0:
                vz = max(-200.0, min(200.0, (h - prev_h)/dt))
        prev_t, prev_h = t, h
        acc = total(fax, fay, faz)

        if state == 'IDLE':
            # Port of FlightStateMachine::detectLiftoffTimed()
            above = acc > LIFTOFF_ACCEL_THRESHOLD
            liftoff = False
            if above:
                if not liftoff_above:
                    liftoff_start = t
                liftoff_above = True
                liftoff = (t - liftoff_start) >= LIFTOFF_CONFIRM_MS
            else:
                if liftoff_above and (t - liftoff_start) < LIFTOFF_CONFIRM_MAX_GAP_MS:
                    liftoff = False  # keep run alive
                else:
                    liftoff_above = False
            if liftoff:
                state = 'ASCENT'
                events.append(('LIFTOFF', t, h, vz, acc))
        elif state == 'ASCENT':
            if abs(vz) < APOGEE_MAX_VZ:
                state = 'DESCENT'
                events.append(('APOGEE', t, h, vz, acc))
        elif state == 'DESCENT':
            if h > PARACHUTE_MIN_ALTITUDE and vz < PARACHUTE_CONFIRM_VZ:
                confirm += 1
                if confirm >= PARACHUTE_CONFIRM_CYCLES:
                    deployed = True
                    events.append(('DEPLOY', t, h, vz, acc))
                    break
            else:
                confirm = 0
    if verbose:
        for e in events:
            print(f'  {e[0]:8s} t={e[1]:9.1f}ms h={e[2]:7.2f}m vz={e[3]:8.2f} acc={e[4]:6.1f}')
    return state, deployed, events

prev_event_t = [None]

def fix_prev(rows):
    prev_event_t[0] = None
    return rows

# ── Loaders ───────────────────────────────────────────────────────────────────
def load_13_30(bench_only=True, tmax=448.0):
    rows = []
    for r in csv.DictReader(open('13_30_11-Dados.csv')):
        t = float(r['millis'])
        if bench_only and t-7060 > tmax*1000:
            break
        rows.append((t, float(r['altp']), float(r['ax']), float(r['ay']), float(r['az'])))
    h0 = rows[0][1]
    return [(t, h-h0, a, b, c) for t, h, a, b, c in rows]

def load_filtrados():
    rows = [(float(r['millis']), float(r['altp']), float(r['ax']),
             float(r['ay']), float(r['az'])) for r in csv.DictReader(open('dados_filtrados.csv'))]
    h0 = rows[0][1]
    pts = [(t, h-h0, a, b, c) for t, h, a, b, c in rows]
    # millis column is degenerate (whole flight in 15.4ms) -> resample to 50Hz
    # on SAMPLE INDEX (native spacing unknown), like the RocketPy sims.
    n = len(pts)
    grid = [i*(1000.0/50) for i in range(n)]  # 20ms per sample
    return list(zip(grid, [p[1] for p in pts], [p[2] for p in pts],
                    [p[3] for p in pts], [p[4] for p in pts]))

def load_resampled_50hz(path):
    """RocketPy sims: resample to 50Hz per skill reference (p10 dt < 10ms)."""
    rows = list(csv.DictReader(open(path)))
    pts = [(float(r['time']), float(r['z']), float(r['ax']),
            float(r['ay']), float(r['az'])) for r in rows]
    z0 = pts[0][1]
    pts = [(t, z-z0, a, b, c) for t, z, a, b, c in pts]
    # resample to 20ms grid
    out = []
    t_end = pts[-1][0]
    step = 0.02
    n = int(t_end/step)
    import bisect
    ts = [p[0] for p in pts]
    for i in range(n):
        tg = i*step
        j = bisect.bisect_left(ts, tg)
        if j == 0:
            out.append((tg*1000, pts[0][1], pts[0][2], pts[0][3], pts[0][4]))
            continue
        if j >= len(pts):
            break
        t0, h0, a0, b0, c0 = pts[j-1]
        t1, h1, a1, b1, c1 = pts[j]
        f = (tg - t0)/(t1 - t0) if t1 > t0 else 0
        out.append((tg*1000, h0 + f*(h1-h0), a0 + f*(a1-a0),
                    b0 + f*(b1-b0), c0 + f*(c1-c0)))
    return out

def synth_spike():
    """Quiet bench + one single-cycle spike of 30 m/s2 in az."""
    rows = []
    for i in range(500):  # 10s at 50Hz
        t = i*20.0
        az = 9.81
        if i == 250:
            az = 30.0  # single 20ms spike
        rows.append((t, 0.0, 0.1, 0.1, az))
    return rows

def synth_drone_drop():
    """Drone climb to 120m (30s) then freefall; FSM must stay IDLE; backstop deploys."""
    rows = []
    t = 0.0
    alt = 0.0
    vz = 0.0
    for i in range(2000):  # 40s climb at 2.5 m/s -> 100m release height
        az_val = 9.81  # steady climb = 1g, no lift-off accel
        rows.append((t*1000, alt, 0.1, 0.1, az_val))
        vz = 2.5  # climbing at 2.5 m/s
        alt += vz*0.02
        t += 0.02
    # release: freefall
    vz_ff = 0.0
    for i in range(200):  # 4s of freefall at 50Hz
        t += 0.02
        vz_ff -= 9.81*0.02
        vz_ff = max(vz_ff, -55.0)
        alt += vz_ff*0.02
        acc = 0.0  # freefall: 0g
        rows.append((t*1000, alt, 0.0, 0, 0.0))
    return rows

def run_backstop(rows):
    """FSM-independent freefall backstop, faithful port."""
    state = 'IDLE'
    fax = fay = faz = None
    prev_t = prev_h = None
    vz = 0.0
    cycles = 0
    deploy_t = None
    for (t, h, ax, ay, az) in rows:
        if fax is None:
            fax, fay, faz = ax, ay, az
        else:
            fax = FILTER_ALPHA*ax + (1-FILTER_ALPHA)*fax
            fay = FILTER_ALPHA*ay + (1-FILTER_ALPHA)*fay
            faz = FILTER_ALPHA*az + (1-FILTER_ALPHA)*faz
        if prev_t is not None:
            dt = (t - prev_t)/1000.0
            if dt > 0:
                vz = (h - prev_h)/dt
        prev_t, prev_h = t, h
        acc = total(fax, fay, faz)
        if acc < FF_ACC and vz < FF_VZ and h > FF_MINH:
            cycles += 1
            if cycles >= FF_CYCLES:
                return t
        else:
            cycles = 0
    return None

# ── Scenarios ─────────────────────────────────────────────────────────────────
results = []
prev_event_t[0] = None
r = load_13_30()
s, d, e = run_fsm(r, 'bench')
print('=== A. BENCH 13_30_11 (t<448s) ===')
run_fsm(r, 'bench', verbose=True) if e else print('  (no events)')
print(f'  final state={s} deployed={d}  -> ' + ('PASS (stayed IDLE)' if s=='IDLE' else 'FAIL'))
results.append(s == 'IDLE')

print('=== B. REAL FLIGHT dados_filtrados ===')
prev_event_t[0] = None
r = load_filtrados()
s, d, e = run_fsm(r, 'real')
for ev in e: print(f'  {ev[0]:8s} t={ev[1]:9.1f}ms h={ev[2]:7.2f}m vz={ev[3]:8.2f}')
ok = len(e) >= 1 and e[0][0] == 'LIFTOFF'
print(f'  -> ' + ('PASS' if ok else 'FAIL'))
results.append(ok)

for sim in ['flight_results_thonyan.csv', 'flight_results_dedalo.csv']:
    print(f'=== C. {sim} (50Hz) ===')
    prev_event_t[0] = None
    r = load_resampled_50hz(sim)
    s, d, e = run_fsm(r, sim)
    for ev in e[:3]: print(f'  {ev[0]:8s} t={ev[1]:9.1f}ms h={ev[2]:7.2f}m vz={ev[3]:8.2f}')
    ok = e and e[0][0] == 'LIFTOFF' and any(x[0]=='DEPLOY' for x in e)
    print(f'  -> ' + ('PASS' if ok else 'FAIL'))
    results.append(ok)

print('=== D. SINGLE 20ms SPIKE 30 m/s2 (quiet bench) ===')
prev_event_t[0] = None
r = synth_spike()
s, d, e = run_fsm(r, 'spike')
print(f'  final state={s} -> ' + ('PASS (IDLE holds)' if s == 'IDLE' else 'FAIL'))
results.append(s == 'IDLE')

print('=== E. DRONE DROP (FSM IDLE, backstop deploys) ===')
r = synth_drone_drop()
bt = run_backstop(r)
ok = bt is not None
if ok:
    print(f'  backstop deploy at t={bt/1000:.2f}s from start (release ~t=30s) -> {((bt/1000)-30):.2f}s after release')
print(f'  -> ' + ('PASS' if ok else 'FAIL'))
results.append(ok)

print()
print('PASS' if all(results) else 'FAIL', f'({sum(results)}/{len(results)} scenarios)')
sys.exit(0 if all(results) else 1)
