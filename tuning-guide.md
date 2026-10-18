# TorqueTwin Bring-Up & Tuning Guide

Sensorless FOC fails in predictable ways. Follow this order; do not skip steps.

## 1. Identify the motor parameters
- **Rs** — multimeter across two phases; phase resistance = reading / 2.
- **Pole pairs** — count magnets / 2, or rotate the shaft one turn and count electrical cycles on a phase-voltage trace.
- **Ld / Lq** — LCR meter at standstill (average several rotor positions). For surface-magnet BLDC, Ld ≈ Lq.
- **Flux** — spin the motor externally, measure peak phase back-EMF E at known electrical speed ω: flux ≈ E / ω.
Put these in `MotorParams M` at the top of the sketch.

## 2. Current-sensor zero
With the bridge disabled, both ACS712 channels must read ≈ 0 A. If not, check the 3.3 V ADC reference and sensor supply (5 V sensor on a 3.3 V ADC needs its output scaled to stay in range).

## 3. Open-loop spin
START with a low speed reference (300 rpm). The motor should ramp smoothly on the I/f start-up. Jerky motion = wrong phase order or wrong pole-pair count.

## 4. Observer lock
Watch the dashboard OBSERVER card. It should switch to LOCKED as the ramp passes ~60%. If it never locks:
- back-EMF too small → raise the ramp speed before handover,
- sliding gain K too low → raise `smo.K` in small steps (too high = noisy angle, audible buzz),
- EMF filter too slow → raise `smo.lpfK` slightly.

## 5. Speed loop
Only now tune `piSpd`. Start with ki = 0, raise kp until the speed follows with a small overshoot, then add ki to remove steady-state droop under load.

## 6. Twin calibration
At a healthy steady run, the twin deviation should sit under ~10%. If it idles higher, the parameters (usually Rs or flux) are off — fix the model, not the threshold.

## 7. MCSA baseline
Record the MCSA fault index on the healthy motor at a few speeds. That baseline — not the textbook table — is what later degradation is judged against.
