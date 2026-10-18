# TorqueTwin — Sensorless FOC Drive + Digital Twin + MCSA 🌀🤖

**Field-Oriented Control of a PMSM/BLDC motor with NO position sensor — rotor angle estimated by a Sliding Mode Observer — plus a real-time digital twin and FFT-based predictive maintenance.**

This is the control technique inside every serious EV drive, taken one level further: the firmware not only runs the motor sensorlessly, it continuously compares the real machine against a physics model of itself (its **digital twin**) and inspects the current spectrum for early fault signatures (**MCSA**).

## 🧠 The three systems

### 1. Sensorless FOC
- Phase currents → **Clarke → Park** transforms into the rotating d-q frame
- PI current loops with d-q decoupling + voltage-circle limiting, **SVPWM** output
- **Sliding Mode Observer (SMO)** estimates back-EMF from the motor model `v = Ri + L·di/dt + e`; a tracking **PLL** converts it into a clean rotor angle and speed
- Open-loop I/f start-up ramp, automatic handover to the observer once back-EMF locks

### 2. Digital twin
- A d-q PMSM model (`Ld/Lq`, flux, inertia, friction) integrates the same voltage commands in parallel
- Sustained model-vs-real current deviation > 25% raises a **twin warning** — the drive notices physical change (demagnetisation, bearing drag, phase resistance rise) before it becomes a failure

### 3. MCSA predictive maintenance
- 1024-point FFT of a phase current at steady speed
- Fault index from the **sidebands around the fundamental** at (1 ± 2s)·f — the classic broken-rotor-bar / eccentricity signature — graded HEALTHY / WATCH / FAULT

## 📂 In this repo

- `TorqueTwin_FOC.ino` — complete firmware (FOC + SMO + PLL, digital twin, MCSA, dashboard)
- `components.md` — component list with the role of each part
- `tuning-guide.md` — motor parameter identification and the tuning sequence that actually works

## 🖥️ Dashboard

Live speed, i_d / i_q, DC-bus voltage, observer lock state, twin deviation %, MCSA verdict, a speed trend chart, speed slider and START/STOP — all served by the ESP32.

## ⚠️ Safety

Real power hardware: develop on a current-limited 24 V-class bench supply, respect the gate driver's own protections, and keep clear of spinning machinery.

## 👤 Author

**Deepesh Chauhan** — B.Tech Electrical Engineering, RGGEC Kangra
[LinkedIn](https://www.linkedin.com/in/deepeshhh1)
