// ============================================================
//  TORQUETWIN v2 - foc.h
//  Field-oriented-control primitives:
//    * PI controller with clamping anti-windup
//    * Clarke / Park / inverse-Park transforms
//    * Voltage-vector (circle) limiter for the d-q commands
//    * SVPWM output via min-max (zero-sequence) injection
//
//  Header-only module: included once by TorqueTwin_FOC.ino.
//  Everything here is pure math plus the final LEDC writes;
//  the control POLICY (ramps, loop order, observer handover)
//  lives in the main sketch.
// ============================================================

#pragma once

#include "config.h"

// =============================================================
//  PI CONTROLLER
//  Discrete PI with conditional-integration anti-windup: when
//  the output saturates, the integral step just taken is
//  rolled back, so the integrator never winds up against the
//  clamp. run() expects the loop period explicitly, which
//  keeps the same gains valid at any control rate.
// =============================================================
struct PI {
  float kp, ki, integral, outMax;

  float run(float err, float dt) {
    integral += err * dt;
    float out = kp * err + ki * integral;
    if (out >  outMax) { out =  outMax; integral -= err * dt; }
    if (out < -outMax) { out = -outMax; integral -= err * dt; }
    return out;
  }
  void reset() { integral = 0; }
};

// =============================================================
//  COORDINATE TRANSFORMS (amplitude-invariant, as v1)
//
//  Clarke: three phase quantities -> stationary alpha/beta.
//  Only ia and ib are measured on this hardware; ic is the
//  computed remainder (-(ia+ib)) and is accepted for clarity.
// =============================================================
inline void clarke(float a, float b, float c, float& al, float& be) {
  (void)c;                                 // balanced set: al = ia
  al = a;
  be = (a + 2.0f * b) * 0.577350269f;      // 1/sqrt(3)
}

// Park: stationary alpha/beta -> rotor-frame d/q at angle th.
inline void park(float al, float be, float th, float& d, float& q) {
  float s = sin(th), co = cos(th);
  d =  al * co + be * s;
  q = -al * s  + be * co;
}

// Inverse Park: rotor-frame d/q -> stationary alpha/beta.
inline void invPark(float d, float q, float th, float& al, float& be) {
  float s = sin(th), co = cos(th);
  al = d * co - q * s;
  be = d * s  + q * co;
}

// =============================================================
//  VOLTAGE CIRCLE LIMITER
//  The inverter can only synthesise |v| <= vbus/sqrt(3) in the
//  linear SVPWM range. If the d-q command exceeds the circle,
//  scale BOTH components by the same factor: magnitude is
//  sacrificed, direction (and therefore torque angle) is not.
//  Returns true when limiting was applied.
// =============================================================
inline bool limitVoltageVector(float& vd, float& vq, float vbusV) {
  float vMax = vbusV * 0.577f;             // 1/sqrt(3)
  float vMag = sqrt(vd * vd + vq * vq);
  if (vMag > vMax && vMag > 0.0f) {
    float s = vMax / vMag;
    vd *= s;
    vq *= s;
    return true;
  }
  return false;
}

// =============================================================
//  SVPWM VIA MIN-MAX (ZERO-SEQUENCE) INJECTION
//  Inverse-Clarke gives the three phase voltage requests; the
//  midpoint of the max/min pair is subtracted as a common
//  (zero-sequence) offset. This centres the duty cycles and
//  reaches the same modulation depth as classic sector-based
//  SVPWM without sector logic.
// =============================================================
inline float focDutyToCounts(float dutyFrac) {
  float d = constrain(dutyFrac, DUTY_MIN, DUTY_MAX);
  return d * PWM_DUTY_MAX;
}

inline void svpwm(float al, float be, float vbusV) {
  float va = al;
  float vb = -0.5f * al + 0.866025404f * be;   // cos(120) / sin(120)
  float vc = -0.5f * al - 0.866025404f * be;

  float vmax = max(va, max(vb, vc));
  float vmin = min(va, min(vb, vc));
  float vzs  = -0.5f * (vmax + vmin);          // zero-sequence shift

  float scale = (vbusV > 1.0f) ? (1.0f / vbusV) : 0.0f;
  ledcWrite(PWM_CH_A, (uint32_t)focDutyToCounts(0.5f + (va + vzs) * scale));
  ledcWrite(PWM_CH_B, (uint32_t)focDutyToCounts(0.5f + (vb + vzs) * scale));
  ledcWrite(PWM_CH_C, (uint32_t)focDutyToCounts(0.5f + (vc + vzs) * scale));
}

// All phases off - used whenever the drive is disabled so the
// bridge never free-runs on a stale duty cycle.
inline void pwmOff() {
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
  ledcWrite(PWM_CH_C, 0);
}

// =============================================================
//  ELECTRICAL <-> MECHANICAL HELPERS
// =============================================================
inline float rpmToElecRadS(float rpm) {
  return rpm * 2.0f * PI / 60.0f * M.polePairs;
}
inline float elecRadSToRpm(float omegaElec) {
  return omegaElec / (2.0f * PI) * 60.0f / M.polePairs;
}

// =============================================================
//  D-Q DECOUPLING FEED-FORWARD
//  In the rotor frame the axes are cross-coupled:
//    v_d = R*i_d + Ld*di_d/dt - w*Lq*i_q
//    v_q = R*i_q + Lq*di_q/dt + w*(Ld*i_d + flux)
//  Feeding the coupling terms forward lets the two PI loops
//  each see an almost pure first-order RL plant, which is
//  what makes one gain set work across the whole speed range.
//  focStep adds these terms to the PI outputs.
// =============================================================
inline void dqDecoupling(float w, float idA, float iqA,
                         float& vdFF, float& vqFF) {
  vdFF = -w * M.Lq * iqA;
  vqFF =  w * (M.Ld * idA + M.flux);
}

// =============================================================
//  OPEN-LOOP START-UP RAMP (I/f)
//  A back-EMF observer is blind at standstill: there is no
//  signal to observe. The standard cure is to drag the rotor
//  around open-loop - apply a rotating frame whose speed ramps
//  linearly from 0 to the reference over RAMP_TIME_S - until
//  the back-EMF is large enough for the observer to lock, then
//  hand over. This struct owns that ramp state so focStep only
//  asks: step(), theta, fraction.
// =============================================================
struct OpenLoopRamp {
  float theta    = 0;   // open-loop electrical angle (rad)
  float fraction = 0;   // 0..1 progress along the ramp

  // Advance the ramp one control step toward omegaRefE
  // (electrical rad/s at full speed). The applied frame speed
  // is omegaRefE * fraction, so the angle accelerates gently
  // instead of jumping to full speed in one step.
  void step(float omegaRefE, float dt) {
    fraction = min(1.0f, fraction + dt / RAMP_TIME_S);
    theta += omegaRefE * fraction * dt;
    while (theta > PI) theta -= 2 * PI;
  }

  // Electrical speed the frame is currently rotating at.
  float omegaNow(float omegaRefE) const {
    return omegaRefE * fraction;
  }

  void reset() {
    theta = 0;
    fraction = 0;
  }
};
