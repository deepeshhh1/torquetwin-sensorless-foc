# Components — TorqueTwin Sensorless FOC Drive

| S.No. | Component | Purpose | Qty |
| --- | --- | --- | --- |
| 1 | ESP32 DevKit V1 (30-pin) | FOC + observer + twin compute, Wi-Fi dashboard | 1 |
| 2 | 3-Phase Gate Driver Board (e.g. DRV8302 module) | Drives the MOSFET bridge, hardware protection | 1 |
| 3 | BLDC / PMSM Motor (24 V class, sensored optional for bring-up) | The machine under control | 1 |
| 4 | ACS712 Current Sensors (5 A) | Phase A and B current measurement (C is computed) | 2 |
| 5 | Voltage Divider Network | DC-bus voltage sensing | 1 set |
| 6 | Bench DC Supply 24 V, current-limited | Safe development power | 1 |
| 7 | Bulk Capacitor (470–1000 µF, 35 V+) | DC-bus smoothing at the bridge | 1 |
| 8 | Coupling + Small Load (fan / disc) | Visible, safe mechanical load | 1 set |
| 9 | 5 V 2 A Adapter (logic) | ESP32 supply (isolated from bus where practical) | 1 |

**Bring-up order matters:** sensor offsets first, open-loop spin second, observer lock third, closed speed loop last — see `tuning-guide.md`.
