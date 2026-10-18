// ============================================================
//  TORQUETWIN v2 - observer.h
//  Sliding Mode Observer (SMO) for sensorless rotor position.
//
//  The stator equation in the stationary alpha-beta frame is
//       v = R*i + L*di/dt + e
//  where e is the rotor back-EMF, perpendicular to the rotor
//  flux. The observer integrates a copy of that equation and
//  drives its current estimate onto the measured current with
//  a discontinuous (sliding) correction term:
//
//       L*d(iHat)/dt = v - R*iHat - z
//       z = K * sign(iHat - i)
//
//  In sliding motion, the low-passed correction term z IS the
//  back-EMF estimate e. A tracking PLL then converts the
//  back-EMF vector into a smooth angle and speed, rejecting
//  the chattering the raw atan2() of a sliding observer has.
//
//  Self-contained: gains and thresholds come from config.h,
//  the motor model comes from the shared MotorParams object M.
//
//  TUNING NOTES
//    * K must exceed the largest back-EMF magnitude the drive
//      will produce, with margin. Too small: the observer
//      drops out at speed. Too large: angle noise (audible
//      buzz). Adjust in ~20% steps from the config value.
//    * The PLL gains set a ~15 Hz tracking bandwidth: fast
//      enough to follow real acceleration, slow enough to
//      reject the sliding observer's chattering residue.
// ============================================================

#pragma once

#include "config.h"

// Wrap an angle to (-PI, PI] - used for PLL phase errors and
// for the tracked angle itself.
static inline float wrapPi(float a) {
  while (a >  PI) a -= 2 * PI;
  while (a < -PI) a += 2 * PI;
  return a;
}

struct SMO {
  // ---- observer state --------------------------------------
  float iAlHat = 0, iBeHat = 0;   // estimated alpha/beta currents
  float eAl = 0, eBe = 0;         // estimated back-EMF vector

  // ---- tuning (defaults from config.h) ---------------------
  float K     = SMO_K_GAIN;       // sliding gain (V)
  float lpfK  = SMO_EMF_LPF_K;    // back-EMF LPF blend @5 kHz
  float pllKp = SMO_PLL_KP;       // tracking PLL proportional
  float pllKi = SMO_PLL_KI;       // tracking PLL integral

  // Optional boundary layer: false (default) = pure sliding
  // mode exactly as the classic observer; true = proportional
  // inside +/- SMO_BOUNDARY_A of current error, which audibly
  // reduces chattering at the cost of a small estimation lag.
  bool boundaryLayer = false;

  // ---- tracking-PLL state ----------------------------------
  float pllInt = 0;
  float theta  = 0;               // estimated electrical angle
  float omega  = 0;               // estimated electrical speed

  // ---- lock reporting --------------------------------------
  bool  locked      = false;      // true while |e| is tracked
  float emfMag      = 0;          // last |e| estimate (V)
  float lockQuality = 0;          // 0..100, smoothed PLL health

  // ------------------------------------------------------------
  //  One observer step. Call at the control rate with the
  //  COMMANDED alpha/beta voltages (what the inverter was
  //  asked to apply) and the MEASURED alpha/beta currents.
  // ------------------------------------------------------------
  // Sliding correction for one axis: the sign of the current
  // estimation error, scaled by the sliding gain - or, with the
  // boundary layer enabled, a saturated proportional term.
  float slideTerm(float err) const {
    if (!boundaryLayer) return K * (err > 0 ? 1.0f : -1.0f);
    float s = err / SMO_BOUNDARY_A;
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;
    return K * s;
  }

  void step(float vAl, float vBe, float iAl, float iBe, float dt) {
    // Sliding correction: bang-bang term on the current error.
    float zAl = slideTerm(iAlHat - iAl);
    float zBe = slideTerm(iBeHat - iBe);

    // Current-estimate integration (forward Euler at dt).
    iAlHat += (dt / M.Ld) * (vAl - M.Rs * iAlHat - zAl);
    iBeHat += (dt / M.Ld) * (vBe - M.Rs * iBeHat - zBe);

    // Back-EMF = low-passed sliding term. lpfK is a per-sample
    // blend, already re-scaled for the 5 kHz rate in config.h.
    eAl += lpfK * (zAl - eAl);
    eBe += lpfK * (zBe - eBe);

    emfMag = sqrt(eAl * eAl + eBe * eBe);

    if (emfMag > SMO_EMF_LOCK_V) {
      // Enough back-EMF to track. For this back-EMF orientation
      // the flux angle is atan2(-eAl, eBe).
      float rawTheta = atan2f(-eAl, eBe);
      float err = wrapPi(rawTheta - theta);

      // Tracking PLL: PI loop filter on the phase error.
      pllInt += err * dt;
      omega   = pllKp * err + pllKi * pllInt;
      theta   = wrapPi(theta + omega * dt);
      locked = true;

      // Lock quality: 100 when the PLL phase error is zero,
      // degrading ~3 points per degree of steady error,
      // smoothed so the dashboard readout does not jitter.
      float errDeg = fabs(err) * 180.0f / PI;
      float q = constrain(100.0f - errDeg * 3.0f, 0.0f, 100.0f);
      lockQuality += 0.05f * (q - lockQuality);
    } else {
      // Signal too weak (standstill / very low speed): the
      // drive must stay on its open-loop ramp. Quality decays
      // so a stale "good lock" is never displayed.
      locked = false;
      lockQuality += 0.05f * (0.0f - lockQuality);
    }
  }

  // Zero all observer state - called while the drive is
  // disabled so a restart never inherits a stale angle, a
  // wound-up PLL integrator, or a phantom lock.
  void reset() {
    iAlHat = iBeHat = 0;
    eAl = eBe = 0;
    pllInt = 0;
    theta = 0;
    omega = 0;
    locked = false;
    emfMag = 0;
    lockQuality = 0;
  }

  // Estimated mechanical speed in rpm (dashboard convenience).
  float speedRPM() const {
    return omega / (2.0f * PI) * 60.0f / M.polePairs;
  }
};
