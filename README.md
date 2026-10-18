# TorqueTwin — Sensorless FOC Drive + Digital Twin + MCSA 🌀🤖

**Field-Oriented Control of a PMSM/BLDC motor with NO position sensor — rotor angle estimated by a Sliding Mode Observer — plus a real-time digital twin and FFT-based predictive maintenance.**

This is the control technique inside every serious EV drive, taken one level further: the firmware not only runs the motor sensorlessly, it continuously compares the real machine against a physics model of itself (its **digital twin**) and inspects the current spectrum for early fault signatures (**MCSA**).

## 🧠 The three systems

### 1. Sensorless FOC
- Phase currents → **Clarke → Park** transforms into the rotating d-q frame
- PI current loops with d-q decoupling + voltage-circle limiting, **SVPWM** output
- **Sliding Mode Observer (SMO)** estimates back-EMF from the motor model `v = Ri + L·di/dt + e`; a tracking **PLL** converts it into a clean rotor angle and speed
- Open-loop I/f start-up ramp, automatic handover to the observer once back-EMF locks
- Control loop runs on a **5 kHz hardware timer**; current-sensor offsets are **auto-calibrated at boot** (300 samples per channel, driver disabled)

### 2. Digital twin
- A d-q PMSM model (`Ld/Lq`, flux, inertia, friction) integrates the same voltage commands in parallel
- Sustained model-vs-real current deviation > 25% raises a **twin warning** — the drive notices physical change (demagnetisation, bearing drag, phase resistance rise) before it becomes a failure

### 3. MCSA predictive maintenance
- 1024-point FFT of a phase current at steady speed
- Fault index from the **sidebands around the fundamental** at (1 ± 2s)·f — the classic broken-rotor-bar / eccentricity signature — graded HEALTHY / WATCH / FAULT, plus 3rd/5th harmonic levels

## 📂 In this repo (multi-file firmware)

- `TorqueTwin_FOC.ino` — main sketch: 5 kHz timer control loop, boot auto-calibration, dashboard
- `config.h` — motor parameters, pins, PWM/control constants, PI + observer gains
- `foc.h` — PI controllers, Clarke/Park transforms, SVPWM, voltage limiting, open-loop ramp
- `observer.h` — sliding-mode observer + tracking PLL (rotor angle/speed estimation)
- `twin.h` — the real-time d-q digital twin and deviation monitor
- `mcsa.h` — FFT current-signature analysis (fault index, harmonics, verdict)
- `components.md` — component list with the role of each part
- `tuning-guide.md` — motor parameter identification and the tuning sequence that actually works

## 🖥️ Dashboard

Live speed, i_d / i_q, DC-bus voltage, observer lock state + lock quality, twin deviation %, MCSA verdict, calibration status, a speed trend chart, speed slider and START/STOP — all served by the ESP32.

## 📚 Required libraries

Board support (once): **esp32 by Espressif Systems** — Arduino IDE → Boards Manager.

| Library | Used for | Install (Library Manager search) |
| --- | --- | --- |
| arduinoFFT (Enrique Condes) | MCSA current-spectrum analysis (1024-point FFT) | `arduinoFFT` |

Built into the ESP32 core, no install needed: `WiFi`, `WebServer`.

## 🔌 Wiring — pin connections (ESP32)

| ESP32 pin | Connects to | Notes |
| --- | --- | --- |
| GPIO25 | Gate driver PWM input — phase A | LEDC channel 0, 20 kHz |
| GPIO26 | Gate driver PWM input — phase B | LEDC channel 1, 20 kHz |
| GPIO27 | Gate driver PWM input — phase C | LEDC channel 2, 20 kHz |
| GPIO14 | Gate driver ENABLE | LOW = driver disabled (boot calibration runs with it LOW) |
| GPIO34 | ACS712 output — phase A current | ADC1 (works with Wi-Fi on) |
| GPIO35 | ACS712 output — phase B current | ADC1 — phase C current is computed (−(ia+ib)) |
| GPIO32 | DC-bus voltage divider output | ADC1 |
| GND | Driver / sensor ground | Common ground throughout |

Gate driver: a 3-phase driver board (e.g. DRV8302-class) between the ESP32 PWM pins and the MOSFET bridge.

## ⚠️ Safety

Real power hardware: develop on a current-limited 24 V-class bench supply, respect the gate driver's own protections, and keep clear of spinning machinery.

## 👤 Author

**Deepesh Chauhan** — B.Tech Electrical Engineering, RGGEC Kangra
[LinkedIn](https://www.linkedin.com/in/deepeshhh1)
