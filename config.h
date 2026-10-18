// ============================================================
//  TORQUETWIN v2 - config.h
//  Central configuration for the TorqueTwin sensorless FOC
//  drive: motor parameters, pin map, sensor scaling, PWM and
//  control-loop timing, and every controller/observer gain.
//
//  All modules (foc.h, observer.h, twin.h, mcsa.h) include this
//  header, so a parameter is changed in exactly ONE place.
//  This is a single-translation-unit Arduino sketch, therefore
//  the motor-parameter object M is defined here (not merely
//  declared) behind the include guard.
//
//  MODULE MAP - who owns what:
//    config.h    this file: every number that might be tuned
//    foc.h       PI, transforms, voltage limiter, SVPWM,
//                d-q decoupling feed-forward, start-up ramp
//    observer.h  SMO + PLL: owns theta/omega estimation and
//                the lock state; nothing else writes them
//    twin.h      PMSM model + deviation judgement; receives
//                the real currents and logger as arguments
//    mcsa.h      self-contained FFT analyser; owns its
//                buffers, its results and its verdict string
//    .ino        hardware setup, calibration, control-step
//                sequencing, dashboard - the only file that
//                touches pins, timers or the web server
//
//  TorqueTwin project - see TorqueTwin_FOC.ino for the full
//  system description and safety notes.
// ============================================================

#pragma once

#include <Arduino.h>
#include <math.h>

// =============================================================
//  MOTOR PARAMETERS (PMSM / BLDC)
//  Identify your own motor before trusting the observer:
//    Rs    - multimeter phase-to-phase / 2
//    Ld/Lq - LCR meter at standstill, several rotor positions
//    flux  - spin test: peak phase back-EMF / electrical speed
//    p     - magnet count / 2
// =============================================================
struct MotorParams {
  float Rs;        // stator resistance, ohm
  float Ld;        // d-axis inductance, H
  float Lq;        // q-axis inductance, H
  float flux;      // rotor flux linkage, Wb
  int   polePairs; // p
  float J;         // rotor inertia, kg.m^2
  float B;         // viscous friction, N.m.s
  float ratedA;    // rated phase current, A
};

// Reference motor: 24 V class PMSM, 4 pole pairs, 8 A rated.
static MotorParams M = {
  0.35f,      // Rs
  0.00045f,   // Ld
  0.00045f,   // Lq (surface-magnet machine: Ld ~= Lq)
  0.0060f,    // flux
  4,          // polePairs
  0.00012f,   // J
  0.00008f,   // B
  8.0f        // ratedA
};

// =============================================================
//  PIN MAP (ESP32 DevKit V1)
// =============================================================
// LEDC PWM outputs to the 3-phase gate driver (e.g. DRV8302)
#define PWM_A_PIN   25
#define PWM_B_PIN   26
#define PWM_C_PIN   27
#define DRV_EN_PIN  14        // gate-driver enable (LOW = disabled)

// LEDC channel numbers paired with the pins above
#define PWM_CH_A    0
#define PWM_CH_B    1
#define PWM_CH_C    2

// Phase-current sensor analog outputs (ADC1 pins only, so the
// ADC keeps working while Wi-Fi is active)
#define IA_PIN      34
#define IB_PIN      35
// DC-bus voltage divider input
#define VBUS_PIN    32

// =============================================================
//  SENSOR SCALING
// =============================================================
static const float CUR_SENS_V_PER_A = 0.185f;  // ACS712 5 A version
static const float ADC_VREF         = 3.3f;    // ESP32 ADC reference
static const float ADC_FULL_SCALE   = 4095.0f; // 12-bit ADC
static const float VBUS_DIV         = 11.0f;   // 100k/10k divider

// =============================================================
//  PWM
// =============================================================
static const int   PWM_FREQ     = 20000;       // 20 kHz switching
static const int   PWM_RES_BITS = 11;          // 0..2047
static const float PWM_DUTY_MAX = 2047.0f;
static const float DUTY_MIN     = 0.02f;       // modulation clamps
static const float DUTY_MAX     = 0.98f;

// =============================================================
//  CONTROL LOOP TIMING
//  v2 runs the FOC step from a hardware-timer flag at 5 kHz
//  (v1 polled micros() at 2 kHz). CTRL_DT is the single source
//  of truth: every integrator, observer and ramp uses it.
// =============================================================
static const float CTRL_DT        = 0.0002f;   // 200 us -> 5 kHz
static const int   CTRL_RATE_HZ   = 5000;
static const float RAMP_TIME_S    = 2.5f;      // open-loop I/f ramp
static const float HANDOVER_RAMP  = 0.6f;      // ramp fraction where
                                               // the observer takes
                                               // over (if locked)

// =============================================================
//  PI GAINS
//  Current loops: output is a voltage command, clamped by the
//  PI's own outMax AND by the SVPWM voltage circle in focStep.
//  Speed loop: output is the iq (torque) reference in amps.
// =============================================================
static const float PI_ID_KP     = 1.2f;
static const float PI_ID_KI     = 900.0f;
static const float PI_ID_OUTMAX = 12.0f;       // V

static const float PI_IQ_KP     = 1.2f;
static const float PI_IQ_KI     = 900.0f;
static const float PI_IQ_OUTMAX = 12.0f;       // V

static const float PI_SPD_KP     = 0.05f;
static const float PI_SPD_KI     = 1.5f;
static const float PI_SPD_OUTMAX = 6.0f;       // A (iq ref pre-clamp;
                                               // focStep also clamps
                                               // to M.ratedA)

// =============================================================
//  SLIDING MODE OBSERVER (observer.h)
//  K and the PLL gains are continuous-time quantities and the
//  observer integrates them with CTRL_DT, so they carry over
//  from the 2 kHz v1 tuning unchanged. The back-EMF low-pass
//  coefficient is a PER-SAMPLE blend factor, so it must scale
//  with the sample period to keep the same filter time
//  constant (~7.8 ms):  alpha = dt / (tau + dt)
//    v1: dt = 500 us -> alpha = 0.060
//    v2: dt = 200 us -> alpha = 0.025
// =============================================================
static const float SMO_K_GAIN     = 9.0f;      // sliding gain (V)
static const float SMO_EMF_LPF_K  = 0.025f;    // back-EMF LPF @5 kHz
static const float SMO_PLL_KP     = 90.0f;     // tracking PLL
static const float SMO_PLL_KI     = 2600.0f;
static const float SMO_EMF_LOCK_V = 0.35f;     // min |e| to track (V)
// Optional boundary layer (observer.h, OFF by default so the
// observer behaves exactly as the classic sliding observer):
// inside +/- this current error the bang-bang term becomes
// proportional, trading a little ideal sliding for much less
// audible chattering on quiet drives.
static const float SMO_BOUNDARY_A = 0.15f;     // A

// =============================================================
//  DIGITAL TWIN (twin.h)
// =============================================================
static const float TWIN_DEV_WARN_PCT = 25.0f;  // deviation threshold
static const unsigned long TWIN_WARN_TIME_MS = 5000; // sustained time
static const float TWIN_MIN_CURRENT_A = 0.3f;  // below this, the
                                               // deviation ratio is
                                               // meaningless noise

// =============================================================
//  CURRENT-OFFSET AUTO-CALIBRATION (.ino, at boot)
//  The ACS712 zero-current output sits near half supply but is
//  never exactly VREF/2; v2 measures it with the driver
//  disabled instead of assuming it.
// =============================================================
static const int CAL_SAMPLES = 300;            // averaged per channel

// =============================================================
//  BRING-UP CHECKLIST (in order, once per new motor)
//   1. Measure Rs / Ld / Lq / flux / pole pairs into the
//      MotorParams object at the top of this file.
//   2. Boot with the bridge disabled and read the CAL log
//      line: both sensor zeros should sit within ~50 mV of
//      each other and close to half supply.
//   3. First spin at DEFAULT_SPEED_RPM on a current-limited
//      supply; LOCK QUALITY should climb past ~70% as the
//      observer hands over.
//   4. Observer never locks? Raise SMO_K_GAIN in ~20% steps
//      (audible buzz = too far), then check phase order.
//   5. Record one MCSA scan of the healthy motor; that
//      baseline is what every later scan is judged against.
// =============================================================

// =============================================================
//  DASHBOARD / OPERATION DEFAULTS
// =============================================================
static const float DEFAULT_SPEED_RPM = 800.0f;
static const float SPEED_MIN_RPM     = 200.0f;
static const float SPEED_MAX_RPM     = 3000.0f;
