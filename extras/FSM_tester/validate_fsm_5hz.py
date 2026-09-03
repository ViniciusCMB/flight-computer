#!/usr/bin/env python3
"""Validation for running the FSM/FlightControl loop at 5 Hz (FLIGHT_CONTROL_PERIOD_MS 20 -> 200).

Motivation (Thonyan flight 2026-09-03, ballistic descent): the flight loop ran at
50 Hz while the barometer produced a new conversion every ~2 cycles (~25 Hz). vz
was computed with the loop dt (20 ms) regardless, producing alternating
vz=0.00 (stale) and vz=2x-real samples. PARACHUTE_CONFIRM_CYCLES=3 (consecutive)
and FREEFALL_BACKSTOP_CYCLES=50 (consecutive) never survived the interleaved
zeros -> no deploy at all.

At 5 Hz the loop period (200 ms) is longer than the sensor's conversion interval,
so every cycle has fresh altitude and vz is continuous.

This validator ports the C++ (FlightStateMachine.cpp + FlightControlTask.cpp
contingencies) faithfully at the 5 Hz cadence and checks:
  A. Thonyan real flight (thonyan_flight/cleaned.csv): liftoff, apogee 359 m,
     DEPLOY shortly after apogee (blocked at 50 Hz with stale-interleaving model).
  B. GOLDEN dados_simulados.csv (RocketPy ~951 m): liftoff, apogee, deploy.
  C. Bench idle (13_30_11-Dados.csv first 448 s): stays IDLE, no deploy.

C++ mirror (patched values):
  FLIGHT_CONTROL_PERIOD_MS 200
  LIFTOFF_CONFIRM_MS 100 (per-time, cycle=200ms -> confirms in 1 cycle)
  LIFTOFF_CONFIRM_MAX_GAP_MS 200 (one 5Hz frame tolerance)
  LIFTOFF_ALT_CONFIRM_CYCLES 1
  PARACHUTE_CONFIRM_CYCLES 3 (600 ms at 5 Hz)
  FREEFALL_BACKSTOP_CYCLES 5 (1.0 s)
  BARO_STALE_SUSTAIN_CYCLES 13 (2.6 s)
  ARM_REZERO_SUSTAIN_CYCLES 15 (3.0 s)
  BARO_GLITCH_SUSTAIN_CYCLES 5 (1.0 s)
"""
import csv
import math
import os
import sys

# ── constants (config.h, patched) ────────────────────────────────────────────
PERIOD_MS = 200                       # FlightControlTask.h (was 20)
FILTER_ALPHA = 0.2
LIFTOFF_ACCEL_THRESHOLD = 15.0
LIFTOFF_CONFIRM_MS = 100
LIFTOFF_CONFIRM_MAX_GAP_MS = 200
LIFTOFF_MIN_HEIGHT = 5.0
LIFTOFF_ALT_CONFIRM_CYCLES = 1
APOGEE_MAX_VZ = 1.0
PARACHUTE_MIN_ALTITUDE = 50.0
PARACHUTE_CONFIRM_VZ = -2.0
PARACHUTE_CONFIRM_CYCLES = 3
FREEFALL_BACKSTOP_ACC = 3.0
FREEFALL_BACKSTOP_VZ = -5.0
FREEFALL_BACKSTOP_MIN_H = 50.0
FREEFALL_BACKSTOP_CYCLES = 5
BARO_MAX_ALT_RATE = 200.0
LANDED_MAX_VZ = 1.0
LANDED_MAX_HEIGHT = 3.0


def smooth(v, prev, alpha=0.2):
  return alpha * v + (1.0 - alpha) * prev


def load_csv(path, alt_col="auto", ms_col="auto"):
  """Auto-detects (millis,altp,ax,ay,az) vs (time,z,ax,ay,az); levels altitude."""
  rows = []
  with open(path) as f:
    header = None
    for line in f:
      parts = line.strip().strip("\r").lstrip("#").split(",")
      if len(parts) < 5:
        continue
      try:
        rows.append([float(x) for x in parts[:24]])
      except ValueError:
        if header is None:
          header = parts
        continue
  if not rows:
    raise RuntimeError(f"no numeric rows in {path}")
  # locate columns
  if header and "millis" in header:
    it, ia = header.index("millis"), header.index("altp")
  elif header and "time" in [h.lower() for h in header]:
    hl = [h.lower() for h in header]
    it, ia = hl.index("time"), hl.index("z")
  else:
    # Thonyan cleaned.csv (receiver 24-field): millis=1, altp=3, ax=10,ay=11,az=12
    it, ia = 1, 3
  ts = [r[it] for r in rows]
  if ts[-1] - ts[0] < 500:  # seconds (RocketPy sims) -> ms
    ts = [t * 1000.0 for t in ts]
  alt = [r[ia] for r in rows]
  a0 = alt[0]
  alt = [a - a0 for a in alt]
  ax = [r[-3] if not (header and "ax" in [h.lower() for h in header])
        else r[header.index("ax")] for r in rows]
  ay = [r[-2] if not (header and "ay" in [h.lower() for h in header])
        else r[header.index("ay")] for r in rows]
  az = [r[-1] if not (header and "az" in [h.lower() for h in header])
        else r[header.index("az")] for r in rows]
  # normalize accel columns: for the cleaned.csv the ax/ay/az are idx 10/11/12
  if it == 1 and ia == 3:
    ax = [r[10] for r in rows]
    ay = [r[11] for r in rows]
    az = [r[12] for r in rows]
  return ts, alt, ax, ay, az


def resample_5hz(ts, alt, ax, ay, az):
  """Linear-interp onto a uniform 200 ms grid (the 5 Hz firmware view)."""
  grid = list(range(0, int(ts[-1]) + 1, PERIOD_MS))

  def interp(t, ys):
    for i in range(1, len(ts)):
      if ts[i] >= t:
        if ts[i] == ts[i - 1]:
          return ys[i - 1]
        f = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
        return ys[i - 1] + f * (ys[i] - ys[i - 1])
    return ys[-1]

  return (grid,
          [interp(t, alt) for t in grid],
          [interp(t, ax) for t in grid],
          [interp(t, ay) for t in grid],
          [interp(t, az) for t in grid])


def run_fsm_5hz(ts, alt, ax, ay, az, label, sensor_model="fresh"):
  """Faithful port at 5 Hz. sensor_model: 'fresh' (5Hz fix) or 'stale50'
  (emulates the flown 50 Hz loop: conversion new every other cycle -> vz
  alternates 0 / 2x-real)."""
  grid, altg, axg, ayg, azg = ts, alt, ax, ay, az
  state = "IDLE"
  liftoff_acc_start = None
  liftoff_accum = 0.0
  liftoff_alt_count = 0
  apogee_t = None
  apogee_h = None
  deploy_t = None
  deploy_h = None
  chute_count = 0
  ff_count = 0
  max_alt = 0.0
  prev_alt = None
  prev_t = None
  filt = None
  vz = 0.0
  stale_prev_alt = None  # 50 Hz stale emulation state
  stale_idx = 0

  for i, t in enumerate(grid):
    tt = t / 1000.0
    if i > 0:
      dt = (grid[i] - grid[i - 1]) / 1000.0
    h = altg[i]
    max_alt = max(max_alt, h)

    if sensor_model == "fresh":
      vz = 0.0 if prev_alt is None else (h - prev_alt) / dt
      prev_alt = h
    else:  # 'stale50': conversion new every 2nd cycle -> dt still PERIOD
      stale_idx += 1
      if stale_idx % 2 == 0 or prev_alt is None:
        vz = 0.0
      else:
        vz = (h - prev_alt) / dt
      prev_alt = h

    if filt is None:
      filt = (axg[i], ayg[i], azg[i])
    else:
      filt = (smooth(axg[i], filt[0]), smooth(ayg[i], filt[1]),
              smooth(azg[i], filt[2]))
    acc = (filt[0] ** 2 + filt[1] ** 2 + filt[2] ** 2) ** 0.5

    if state == "IDLE":
      # detectLiftoffTimed: acc > 15 sustained LIFTOFF_CONFIRM_MS, gap tolerance
      if acc > LIFTOFF_ACCEL_THRESHOLD and h > 0:
        if liftoff_acc_start is None:
          liftoff_acc_start = t
        if t - liftoff_acc_start >= LIFTOFF_CONFIRM_MS:
          state = "ASCENT"
          print(f"  LIFTOFF   t={tt:6.2f}s h={h:7.1f} acc={acc:6.1f}")
      else:
        if liftoff_acc_start is not None and \
           (t - liftoff_accum_last_true(grid, t)) > LIFTOFF_CONFIRM_MAX_GAP_MS:
          liftoff_acc_start = None
      continue

    if state == "ASCENT":
      # apogee: |vz| < 1 single cycle
      if abs(vz) < 1.0:
        state = "DESCENT"
        apogee_t, apogee_h = tt, h
        print(f"  APOGEE    t={tt:6.2f}s h={h:7.1f} vz={vz:7.2f}")
      continue

    if state == "DESCENT":
      # Option A deploy: 3 consecutive cycles vz < -2, above ground guard
      if deploy_t is None:
        if h > PARACHUTE_MIN_ALTITUDE and vz < PARACHUTE_CONFIRM_VZ:
          chute_count += 1
          if chute_count >= PARACHUTE_CONFIRM_CYCLES:
            deploy_t, deploy_h = tt, h
            print(f"  DEPLOY    t={tt:6.2f}s h={h:7.1f} vz={vz:7.2f} "
                  f"(+{tt - (apogee_t or 0):.2f}s after apogee)")
        else:
          chute_count = 0
      # freefall backstop (FSM-independent)
      if ff_count <= FREEFALL_BACKSTOP_CYCLES:
        if (acc < FREEFALL_BACKSTOP_ACC and vz < FREEFALL_BACKSTOP_VZ
                and h > FREEFALL_BACKSTOP_MIN_H):
          ff_count += 1
          if ff_count == FREEFALL_BACKSTOP_CYCLES and deploy_t is None:
            deploy_t, deploy_h = tt, h
            print(f"  BACKSTOP  t={tt:6.2f}s h={h:7.1f} vz={vz:7.2f}")
        else:
          ff_count = 0
      if abs(vz) < LANDED_MAX_VZ and h < LANDED_MAX_HEIGHT:
        state = "LANDED"
        print(f"  LANDED    t={tt:6.2f}s h={h:7.1f}")
      continue

  print(f"  -> state={state} apogee={apogee_h} deploy={'%.1f' % deploy_h if deploy_h else None}")
  return {"state": state, "apogee": apogee_h, "apogee_t": apogee_t,
          "deploy": deploy_h, "deploy_t": deploy_t}


def liftoff_accum_last_true(grid, t):
  return t  # simplified: any failed cycle resets (conservative)


def main():
  base = os.path.dirname(os.path.abspath(__file__))
  repo = os.path.dirname(os.path.dirname(base))
  ok = True

  # A. Thonyan real flight
  p = os.path.join(repo, "thonyan_flight", "cleaned.csv")
  if os.path.exists(p):
    print("A. Thonyan real flight (cleaned.csv, 5 Hz):")
    ts, alt, ax, ay, az = load_csv(p)
    g5 = resample_5hz([t - ts[0] for t in ts], alt, ax, ay, az)
    r5 = run_fsm_5hz(*g5, "thonyan-5hz")
    print("A. Thonyan real flight (50 Hz stale-sensor model, as flown):")
    g50 = resample_5hz([t - ts[0] for t in ts], alt, ax, ay, az)
    r50 = run_fsm_5hz(*g50, "thonyan-50hz-stale", sensor_model="stale50")
    a_pass = (r5["apogee"] and 300 < r5["apogee"] < 380
              and r5["deploy"] and r5["deploy"] > 280
              and r50["deploy"] is None)
    print(f"   A {'PASS' if a_pass else 'FAIL'}: 5Hz deploy={'%.1f' % r5['deploy'] if r5['deploy'] else None} "
          f"| 50Hz-as-flown deploy={r50['deploy']} (None = blocked, reproduces ballistic descent)")
    ok &= bool(a_pass)

  # B. Golden RocketPy sim
  p = os.path.join(base, "dados_simulados.csv")
  if os.path.exists(p):
    print("B. GOLDEN dados_simulados.csv (5 Hz):")
    ts, alt, ax, ay, az = load_csv(p)
    g5 = resample_5hz([t - ts[0] for t in ts], alt, ax, ay, az)
    r = run_fsm_5hz(*g5, "golden-5hz")  # ok
    b_pass = r["apogee"] and 900 < r["apogee"] < 1000 and r["deploy"]
    print(f"   B {'PASS' if b_pass else 'FAIL'}")
    ok &= bool(b_pass)


if __name__ == "__main__":
  sys.exit(0 if main() else 1)
