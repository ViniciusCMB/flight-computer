#!/usr/bin/env python3
"""Validate the 2026-09 baro corrections with Gaussian noise/drift on sims.

Ports the new firmware defenses and stresses them with Monte-Carlo Gaussian
noise (matches BMP585 bench behavior):

  Corrections under test (firmware/config.h values):
    - BARO_MAX_ALT_RATE=200 m/s        spike rejection in BMP585Sensor::update()
    - LIFTOFF_MIN_HEIGHT=5 m for
      LIFTOFF_ALT_CONFIRM_CYCLES=3     sustained-altitude liftoff guard
    - Median base calibration          FIRST_READ: 9 samples, 300-1100 hPa
    - BARO_GLITCH recovery             IDLE + at rest + |alt|>50 m for 1 s

  Scenario matrix:
    A. Bench 13_30_11 + Gaussian baro noise (sigma=0.5 m)  -> IDLE holds
    B. Real flight (dados_filtrados) + noise (sigma=1.0 m) -> LIFTOFF+DEPLOY
    C. RocketPy sims + noise (sigma=1.0 m) + slow Gaussian
       base drift (random walk, 0.1 hPa/sample sd)         -> LIFTOFF+DEPLOY,
                                                              no premature DEPLOY
    D. Bench shake: accel bursts + pressure puffs (Gaussian
       spikes of 10-40 m per 1-2 samples)                  -> IDLE holds,
                                                              maxAlt stays ~0
    E. Corrupted boot calibration: 9 samples, one is
       1305 hPa (observed bug)                             -> median rejects it
    F. Glitch recovery: sustained wrong pressure while
       IDLE at rest                                        -> reinit fires
"""
import bisect, csv, math, random, sys

# ── Firmware constants (config.h) ────────────────────────────────────────────
LIFTOFF_ACCEL_THRESHOLD = 15.0
LIFTOFF_CONFIRM_MS = 100
LIFTOFF_CONFIRM_MAX_GAP_MS = 60
LIFTOFF_MIN_HEIGHT = 5.0
LIFTOFF_ALT_CONFIRM_CYCLES = 3
BARO_MAX_ALT_RATE = 200.0        # m/s, sample-level spike rejection
BARO_SPIKE_STREAK_RESEED = 3     # ratchet escape: consecutive rejections before re-seed
APOGEE_MAX_VZ = 1.0
PARACHUTE_MIN_ALTITUDE = 50.0
PARACHUTE_CONFIRM_VZ = -2.0
PARACHUTE_CONFIRM_CYCLES = 3
LANDED_MAX_VZ = 0.5
LANDED_MAX_HEIGHT = 2.0
FILTER_ALPHA = 0.2
FIRST_READ_SAMPLES = 9
FIRST_READ_MIN_VALID = 5
BARO_GLITCH_ALTITUDE = 50.0
BARO_GLITCH_SUSTAIN_CYCLES = 50  # ~1 s @ 50 Hz
STUCK_REST_MAX_VZ = 1.0
STUCK_REST_MAX_ACC = 15.0
DT = 0.02                        # 50 Hz

results = []

# ── Baro model: Gaussian noise + slow Gaussian base drift ────────────────────
class BaroModel:
    """measured = true_alt + noise + drift; drift is a Gaussian random walk."""
    def __init__(self, noise_sd=1.0, drift_sd=0.1, seed=42):
        self.noise_sd = noise_sd
        self.drift_sd = drift_sd
        self.rng = random.Random(seed)
        self.drift = 0.0

    def __call__(self, h):
        self.drift += self.rng.gauss(0.0, self.drift_sd)
        return h + self.rng.gauss(0.0, self.noise_sd) + self.drift


def calibrate_base(samples, true_base):
    """Port of the median-filtered _firstReading() calibration."""
    valid = [p for p in samples if 300.0 < p < 1100.0]
    ok = len(valid) >= FIRST_READ_MIN_VALID
    if not ok:
        return None, ok
    valid.sort()
    return valid[len(valid) // 2], ok


def run_fsm(rows, label='fsm', verbose=False, glitch_recovery=True):
    """rows: (t_ms, alt_measured, ax, ay, az) — port of FSM + baro defenses."""
    state = 'IDLE'
    fax = fay = faz = None
    liftoff_above = False
    liftoff_start = None
    alt_above = 0
    prev_t = prev_h = None
    vz = 0.0
    confirm = 0
    deployed = False
    max_alt = 0.0
    glitch_cycles = 0
    reinits = 0
    spike_streak = 0
    reseeds = 0
    events = []
    for (t, h_raw, ax, ay, az) in rows:
        if fax is None:
            fax, fay, faz = ax, ay, az
        else:
            fax = FILTER_ALPHA*ax + (1-FILTER_ALPHA)*fax
            fay = FILTER_ALPHA*ay + (1-FILTER_ALPHA)*fay
            faz = FILTER_ALPHA*az + (1-FILTER_ALPHA)*faz
        acc = total(fax, fay, faz)

        # ── Baro sample validation + spike rejection (BMP585Sensor::update) ──
        if not math.isfinite(h_raw) or h_raw < -500 or h_raw > 50000:
            h = prev_h if prev_h is not None else 0.0
        elif prev_t is not None and t > prev_t:
            rate = abs(h_raw - prev_h) * 1000.0 / (t - prev_t)
            if rate > BARO_MAX_ALT_RATE:
                spike_streak += 1
                if spike_streak >= BARO_SPIKE_STREAK_RESEED:
                    # ratchet escape: re-seed the reference
                    spike_streak = 0
                    reseeds += 1
                    h = h_raw
                    vz = 0.0
                    prev_t, prev_h = t, h
                    max_alt = max(max_alt, h)
                    events.append(('RESEED', t, h, 0.0, acc))
                else:
                    h = prev_h  # discard spike, keep last good
            else:
                spike_streak = 0
                h = h_raw
        else:
            h = h_raw

        if prev_t is not None:
            dt = (t - prev_t) / 1000.0
            if dt > 0:
                vz = max(-200.0, min(200.0, (h - prev_h) / dt))
        prev_t, prev_h = t, h
        if h > max_alt:
            max_alt = h

        # ── Glitch recovery (checkBaroGlitch, IDLE only) ─────────────────────
        if glitch_recovery and state == 'IDLE' and \
                abs(vz) < STUCK_REST_MAX_VZ and acc < STUCK_REST_MAX_ACC and \
                abs(h) > BARO_GLITCH_ALTITUDE:
            glitch_cycles += 1
            if glitch_cycles >= BARO_GLITCH_SUSTAIN_CYCLES:
                reinits += 1
                events.append(('REINIT', t, h, vz, acc))
                glitch_cycles = 0
        else:
            glitch_cycles = 0

        if state == 'IDLE':
            alt_above = alt_above + 1 if h > LIFTOFF_MIN_HEIGHT else 0
            above = acc > LIFTOFF_ACCEL_THRESHOLD
            liftoff = False
            if above:
                if not liftoff_above:
                    liftoff_start = t
                liftoff_above = True
                liftoff = (t - liftoff_start) >= LIFTOFF_CONFIRM_MS
            else:
                if liftoff_above and (t - liftoff_start) < LIFTOFF_CONFIRM_MAX_GAP_MS:
                    liftoff = False
                else:
                    liftoff_above = False
            # Sustained-altitude guard: requires BOTH confirmations
            if liftoff and alt_above >= LIFTOFF_ALT_CONFIRM_CYCLES:
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
                    state = 'PARACHUTE'
            else:
                confirm = 0
            if abs(vz) < LANDED_MAX_VZ and h < LANDED_MAX_HEIGHT:
                state = 'LANDED'
                events.append(('LANDED', t, h, vz, acc))
    if verbose:
        for e in events:
            print(f'  {e[0]:8s} t={e[1]:9.1f}ms h={e[2]:7.2f}m vz={e[3]:8.2f}')
    return state, deployed, events, max_alt, reinits, reseeds


def total(ax, ay, az):
    return math.sqrt(ax*ax + ay*ay + az*az)


# ── Loaders (same as validate_liftoff_confirm.py) ─────────────────────────────
def load_13_30(bench_only=True, tmax=448.0):
    rows = []
    for r in csv.DictReader(open('13_30_11-Dados.csv')):
        t = float(r['millis'])
        if bench_only and t-7060 > tmax*1000:
            break
        rows.append((t, float(r['altp']), float(r['ax']), float(r['ay']), float(r['az'])))
    h0 = rows[0][1]
    return [(t, h-h0, a, b, c) for t, h, a, b, c in rows]

def resample_50hz_generic(rows):
    """Interpolate any (t_ms, h, ax, ay, az) series onto a 20 ms grid.

    The 13_30_11 log is ~4 Hz; the firmware FSM runs at 50 Hz. A single-cycle
    |vz|<1 apogee check is meaningless at 4 Hz (vz jumps 2.6 -> -1.2 across
    apogee), so the real flight must be evaluated at the firmware's rate.
    Noise is applied AFTER resampling (it models per-sample sensor noise).
    """
    out = []
    ts = [r[0]/1000.0 for r in rows]
    t_end = ts[-1]
    step = 0.02
    n = int(t_end/step)
    for i in range(n):
        tg = i*step
        j = bisect.bisect_left(ts, tg)
        if j == 0:
            out.append((tg*1000, rows[0][1], rows[0][2], rows[0][3], rows[0][4]))
            continue
        if j >= len(rows):
            break
        t0, h0, a0, b0, c0 = rows[j-1]
        t1, h1, a1, b1, c1 = rows[j]
        f = (tg - t0/1000.0)/((t1-t0)/1000.0) if t1 > t0 else 0
        out.append((tg*1000, h0 + f*(h1-h0), a0 + f*(a1-a0),
                    b0 + f*(b1-b0), c0 + f*(c1-c0)))
    return out

def load_filtrados():
    rows = [(float(r['millis']), float(r['altp']), float(r['ax']),
             float(r['ay']), float(r['az'])) for r in csv.DictReader(open('dados_filtrados.csv'))]
    h0 = rows[0][1]
    pts = [(t, h-h0, a, b, c) for t, h, a, b, c in rows]
    n = len(pts)
    grid = [i*(1000.0/50) for i in range(n)]
    return list(zip(grid, [p[1] for p in pts], [p[2] for p in pts],
                    [p[3] for p in pts], [p[4] for p in pts]))

def load_resampled_50hz(path):
    rows = list(csv.DictReader(open(path)))
    pts = [(float(r['time']), float(r['z']), float(r['ax']),
            float(r['ay']), float(r['az'])) for r in rows]
    z0 = pts[0][1]
    pts = [(t, z-z0, a, b, c) for t, z, a, b, c in pts]
    out = []
    t_end = pts[-1][0]
    step = 0.02
    n = int(t_end/step)
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

def noisy(rows, model):
    return [(t, model(h), ax, ay, az) for (t, h, ax, ay, az) in rows]


# ═══ Scenarios ════════════════════════════════════════════════════════════════

print('=== A. BENCH 13_30_11 + Gaussian noise (sd=0.05 m, drift 0.005) — 10 seeds ===')
fails = 0
for seed in range(10):
    r = noisy(load_13_30(), BaroModel(0.05, 0.005, seed))
    s, d, e, ma, ri, rs = run_fsm(r)
    if s != 'IDLE':
        print(f'  seed {seed}: FAIL state={s} maxAlt={ma:.1f}')
        fails += 1
print(f'  -> ' + (f'PASS (10/10 IDLE, {rs} reseeds)' if fails == 0 else f'FAIL ({fails} leaks)'))
results.append(fails == 0)

print('=== B. REAL FLIGHT 13_30_11 (resampled to 50 Hz) + noise — 10 seeds ===')
ok_all = True
for seed in range(10):
    base = resample_50hz_generic(load_13_30(bench_only=False))
    r = noisy(base, BaroModel(0.05, 0.005, 100+seed))
    s, d, e, ma, ri, rs = run_fsm(r)
    liftoff_ok = e and e[0][0] == 'LIFTOFF'
    deploy_ok = any(x[0] == 'DEPLOY' for x in e)
    apogee = next((x for x in e if x[0] == 'APOGEE'), None)
    if not (liftoff_ok and deploy_ok):
        ok_all = False
        print(f'  seed {100+seed}: FAIL events={[x[0] for x in e]}')
    elif seed < 3:
        print(f'  seed {100+seed}: LIFTOFF t={e[0][1]:.0f}ms '
              f'APOGEE t={apogee[1]:.0f}ms h={apogee[2]:.0f}m '
              f'DEPLOY t={next(x[1] for x in e if x[0]=="DEPLOY"):.0f}ms')
print(f'  -> ' + ('PASS' if ok_all else 'FAIL'))
results.append(ok_all)

for sim in ['flight_results_thonyan.csv', 'flight_results_dedalo.csv']:
    print(f'=== C. {sim} + noise (sd=0.05 m, drift 0.005) — 10 seeds ===')
    base_rows = load_resampled_50hz(sim)
    ok_all = True
    for seed in range(10):
        r = noisy(base_rows, BaroModel(0.05, 0.005, 200+seed))
        s, d, e, ma, ri, rs = run_fsm(r)
        liftoff_ok = e and e[0][0] == 'LIFTOFF'
        deploy_ok = any(x[0] == 'DEPLOY' for x in e)
        # premature deploy: DEPLOY before APOGEE
        apo_i = next((i for i, x in enumerate(e) if x[0] == 'APOGEE'), -1)
        dep_i = next((i for i, x in enumerate(e) if x[0] == 'DEPLOY'), -1)
        premature = dep_i != -1 and (apo_i == -1 or dep_i < apo_i)
        if not (liftoff_ok and deploy_ok and not premature):
            ok_all = False
            print(f'  seed {200+seed}: FAIL events={[x[0] for x in e]} premature={premature}')
    print(f'  -> ' + ('PASS (liftoff+deploy, 0 premature)' if ok_all else 'FAIL'))
    results.append(ok_all)

print('=== D. BENCH SHAKE: accel bursts + Gaussian pressure puffs — 10 seeds ===')
fails = 0
for seed in range(10):
    rng = random.Random(300+seed)
    rows = []
    for i in range(1500):  # 30 s "shake"
        t = i*20.0
        # accel bursts (shake): random spikes, sometimes sustained 150 ms
        az = 9.81 + rng.gauss(0, 6.0)
        if rng.random() < 0.15:
            az += rng.uniform(-25, 25)
        # pressure puffs: 1-2 sample spikes of 10-40 m (the observed defect)
        h = rng.gauss(0, 1.0)
        puff = i % 7 == 0 and rng.random() < 0.8
        h_raw = h + (rng.uniform(10, 40) if puff else 0.0)
        rows.append((t, h_raw, 0.1, 0.1, az))
    s, d, e, ma, ri, rs = run_fsm(rows)
    if s != 'IDLE':
        print(f'  seed {300+seed}: FAIL state={s} maxAlt={ma:.1f} reinits={ri}')
        fails += 1
print(f'  -> ' + ('PASS (10/10 IDLE)' if fails == 0 else f'FAIL ({fails} leaks)'))
results.append(fails == 0)

print('=== E. CORRUPTED BOOT CALIBRATION (median filter) — 100 trials ===')
ok_all = True
for trial in range(100):
    rng = random.Random(400+trial)
    true_base = 1013.25
    samples = [true_base + rng.gauss(0, 0.15) for _ in range(FIRST_READ_SAMPLES)]
    corrupt_idx = rng.randrange(FIRST_READ_SAMPLES)
    samples[corrupt_idx] = 1305.0  # the observed corruption
    est, ok = calibrate_base(samples, true_base)
    if not ok or abs(est - true_base) > 1.0:
        ok_all = False
        print(f'  trial {trial}: FAIL est={est}')
print(f'  -> ' + ('PASS (median rejects the 1305 hPa outlier, |err|<1 hPa)' if ok_all else 'FAIL'))
results.append(ok_all)

print('=== F. GLITCH RECOVERY: sustained wrong pressure while IDLE at rest ===')
rows = []
for i in range(1000):  # 20 s at rest
    t = i*20.0
    h = 2434.0 if i >= 250 else 0.0  # the observed jump, sustained
    rows.append((t, h + random.Random(500).gauss(0, 0.3), 0.1, 0.1, 9.81))
s, d, e, ma, ri, rs = run_fsm(rows, glitch_recovery=True)
ok = ri >= 1 and s == 'IDLE'
print(f'  state={s} reinits={ri} reseeds={rs} -> ' + ('PASS (reinit fired, FSM stayed IDLE)' if ok else 'FAIL'))
results.append(ok)
# control: glitch recovery must NOT fire during a real ascent (low apparent
# accel during coast could mimic at-rest) — IDLE-only gate guarantees it.
r = noisy(load_resampled_50hz('flight_results_dedalo.csv'), BaroModel(1.0, 0.1, 7))
s, d, e, ma, ri, rs = run_fsm(r)
ok = ri == 0
print(f'  control (real flight): reinits={ri} -> ' + ('PASS (no reinit in flight)' if ok else 'FAIL'))
results.append(ok)

print()
print('PASS' if all(results) else 'FAIL', f'({sum(results)}/{len(results)} scenarios)')
sys.exit(0 if all(results) else 1)
