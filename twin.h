// ============================================================
//  TORQUETWIN v2 - twin.h
//  Digital twin of the PMSM, integrated in parallel with the
//  real drive on the same voltage commands:
//
//    di_d/dt = (v_d - R*i_d + w*Lq*i_q) / Ld
//    di_q/dt = (v_q - R*i_q - w*(Ld*i_d + flux)) / Lq
//    dw/dt   = (Te - Tl - B*w) / J
//    Te      = 1.5 * p * (flux*i_q + (Ld - Lq)*i_d*i_q)
//
//  The twin sees the COMMANDED voltages; the real machine is
//  observed through its measured d-q currents. If the two
//  current magnitudes diverge by more than TWIN_DEV_WARN_PCT
//  for longer than TWIN_WARN_TIME_MS while the drive runs,
//  something physical has changed (demagnetisation, rising
//  phase resistance, bearing drag, a loosening connector):
//  the twin raises a warning BEFORE the change becomes a
//  failure.
//
//  The class is deliberately free of sketch globals: the real
//  currents, the drive-enabled flag and the event logger are
//  passed into step(), which keeps the twin unit-testable and
//  the module self-contained.
// ============================================================

#pragma once

#include "config.h"

// Event logger signature provided by the main sketch.
typedef void (*TwinLogger)(const String& msg);

struct Twin {
  // ---- twin state ------------------------------------------
  float idT = 0, iqT = 0;         // twin d-q currents (A)
  float wmT = 0;                  // twin mechanical speed (rad/s)

  // External load torque applied to the twin's mechanical
  // model (N.m). Zero by default = the v1 no-load behaviour.
  // Set it (directly, or with the fan-load helper below) when
  // an experiment wants the twin to work against the same load
  // the real machine sees, e.g. a fan whose torque grows with
  // speed squared.
  float loadTorque = 0;

  // ---- health reporting ------------------------------------
  float deviationPct = 0;         // | |iReal| - |iTwin| | / |iReal|
  bool  warning      = false;     // latched while deviation lasts
  unsigned long warnSince = 0;    // first ms the threshold broke

  // ------------------------------------------------------------
  //  One twin step.
  //    vdIn / vqIn   - d-q voltages commanded this step
  //    omegaElec     - estimated electrical speed of the REAL
  //                    machine (observer), used as the model's
  //                    speed reference
  //    idReal/iqReal - measured d-q currents of the real machine
  //    driveOn       - deviation is only judged while running
  //    dt            - control period (CTRL_DT)
  //    logger        - sketch logEvent(), or nullptr
  // ------------------------------------------------------------
  void step(float vdIn, float vqIn, float omegaElec,
            float idReal, float iqReal, bool driveOn, float dt,
            TwinLogger logger) {
    float w = omegaElec;
    idT += (dt / M.Ld) * (vdIn - M.Rs * idT + w * M.Lq * iqT);
    iqT += (dt / M.Lq) * (vqIn - M.Rs * iqT - w * (M.Ld * idT + M.flux));

    // Mechanical side: electromagnetic torque against friction
    // and the (normally zero) modelled load torque. The twin
    // judges the ELECTRICAL match, which is where faults show
    // first, so the load model stays deliberately simple.
    float te = torque();
    wmT += (dt / M.J) * (te - loadTorque - M.B * wmT);

    // ---- deviation judgement --------------------------------
    float realMag = sqrt(idReal * idReal + iqReal * iqReal);
    float twinMag = sqrt(idT * idT + iqT * iqT);
    deviationPct = (realMag > TWIN_MIN_CURRENT_A)
        ? fabs(realMag - twinMag) / realMag * 100.0f : 0.0f;

    if (driveOn && deviationPct > TWIN_DEV_WARN_PCT) {
      if (warnSince == 0) warnSince = millis();
      if (millis() - warnSince > TWIN_WARN_TIME_MS && !warning) {
        warning = true;
        if (logger) {
          logger("TWIN WARNING: model/real current deviation " +
                 String(deviationPct, 0) +
                 "% sustained - inspect motor/drive");
        }
      }
    } else {
      warnSince = 0;
      warning = false;
    }
  }

  // Electromagnetic torque the twin is currently producing.
  float torque() const {
    return 1.5f * M.polePairs *
           (M.flux * iqT + (M.Ld - M.Lq) * idT * iqT);
  }

  // Twin current magnitude (dashboard / diagnostics).
  float currentMag() const {
    return sqrt(idT * idT + iqT * iqT);
  }

  // Convenience load models for experiments. fanLoad() sets
  // the classic quadratic fan/pump torque Tl = k * wm^2 using
  // the twin's own speed; call it once per control step (or
  // whenever the operating point changes) before step().
  void fanLoad(float k) {
    loadTorque = k * wmT * fabs(wmT);
  }
  void constantLoad(float nm) {
    loadTorque = nm;
  }

  // Zero the model - e.g. after a parameter change. (Normal
  // stop/start does NOT reset the twin: its currents decay to
  // the real ones naturally through the model equations.)
  void reset() {
    idT = iqT = 0;
    wmT = 0;
    deviationPct = 0;
    warning = false;
    warnSince = 0;
  }
};
