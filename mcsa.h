// ============================================================
//  TORQUETWIN v2 - mcsa.h
//  Motor Current Signature Analysis (MCSA).
//
//  At steady speed, one phase current is captured and FFT'd.
//  A healthy machine shows essentially only the supply
//  fundamental. Rotor and bearing defects modulate the air-gap
//  flux and grow SIDEBANDS around that fundamental at
//  (1 +/- 2s)*f1, where s is the slip. The fault index used
//  here is the level of the strongest sideband BELOW the
//  fundamental, in dB - the classic MCSA health number:
//
//      > 50 dB   healthy
//      40-50 dB  watch - sidebands rising
//      < 40 dB   fault signature - inspect rotor / bearings
//
//  Requires the "arduinoFFT" library by Enrique Condes.
//
//  NOTE ON EXECUTION CONTEXT: run() blocks for ~0.5 s while it
//  samples. Call it from loop()/web-handler context only,
//  never from the control step - the main sketch accounts for
//  the dropped control ticks in its overrun counter.
// ============================================================

#pragma once

#include "config.h"
#include <arduinoFFT.h>

// Event logger signature provided by the main sketch.
typedef void (*MCSAlogger)(const String& msg);

// Phase-current sampler signature: the sketch's calibrated
// readPhaseCurrent(int pin) matches this exactly.
typedef float (*CurrentSampler)(int pin);

class MCSAAnalyzer {
public:
  static constexpr uint16_t FFT_N       = 1024;  // samples per scan
  static constexpr float    SAMPLE_RATE = 2000.0f; // Hz

  // ---- results (read by the dashboard after a scan) --------
  float  faultIndexDb = 0;        // sideband depth below fund.
  String verdict      = "NOT ANALYSED";
  float  fundamentalHz = 0;       // located fundamental (Hz)
  float  harm3Db = 0;             // 3rd harmonic below fund. (dB)
  float  harm5Db = 0;             // 5th harmonic below fund. (dB)
  bool   busy          = false;   // a scan is in progress

  MCSAAnalyzer()
      : fft(vReal, vImag, FFT_N, SAMPLE_RATE) {}

  // ------------------------------------------------------------
  //  Run one scan.
  //    omegaElec - estimated electrical speed (rad/s) of the
  //                machine; locates the fundamental bin
  //    sampler   - calibrated phase-current reader (phase A)
  //    logger    - sketch logEvent(), or nullptr
  //  Returns true if a scan was performed and graded, false
  //  if it was skipped (already busy, or the fundamental fell
  //  outside the analysable band).
  // ------------------------------------------------------------
  bool run(float omegaElec, CurrentSampler sampler, MCSAlogger logger) {
    if (busy) return false;
    busy = true;

    // --- capture one phase current at SAMPLE_RATE -----------
    const uint32_t samplePeriodUs = (uint32_t)(1000000.0f / SAMPLE_RATE);
    for (uint16_t i = 0; i < FFT_N; i++) {
      vReal[i] = sampler(IA_PIN);
      vImag[i] = 0;
      delayMicroseconds(samplePeriodUs);
    }

    // --- FFT -------------------------------------------------
    fft.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    fft.compute(FFTDirection::Forward);
    fft.complexToMagnitude();          // magnitudes now in vReal

    // --- locate the fundamental ------------------------------
    double speedHz = omegaElec / (2.0 * PI);   // electrical freq
    fundamentalHz = (float)speedHz;
    int fundBin = (int)(speedHz * FFT_N / SAMPLE_RATE);
    if (fundBin < 3 || fundBin > (int)FFT_N / 2 - 8) {
      busy = false;
      return false;                    // outside analysable band
    }
    double fundMag = 0;
    for (int b = fundBin - 2; b <= fundBin + 2; b++)
      fundMag = max(fundMag, vReal[b]);

    // --- sideband search at (1 +/- 2s) * f1 -------------------
    // Slip is not measured directly on this drive; the demo
    // value 3% places the search where broken-bar sidebands
    // appear on a loaded machine. The search takes the worst
    // (largest) sideband within +/-1 bin of each expected spot.
    const float slip = 0.03f;
    int sbOffset = max(2, (int)(2.0f * slip * speedHz * FFT_N / SAMPLE_RATE));
    double sbMag = 0;
    for (int b = fundBin - sbOffset - 1; b <= fundBin - sbOffset + 1; b++)
      if (b > 0) sbMag = max(sbMag, vReal[b]);
    for (int b = fundBin + sbOffset - 1; b <= fundBin + sbOffset + 1; b++)
      if (b < (int)FFT_N / 2) sbMag = max(sbMag, vReal[b]);

    // --- harmonic levels --------------------------------------
    // Supply distortion and saturation show up as odd harmonics
    // of the fundamental. Reported alongside the MCSA index so
    // a "fault" verdict can be sanity-checked: strong sidebands
    // with CLEAN harmonics point at the machine; strong
    // harmonics point at the supply or the inverter instead.
    harm3Db = harmonicDb(fundBin * 3, fundMag);
    harm5Db = harmonicDb(fundBin * 5, fundMag);

    // --- grade -------------------------------------------------
    faultIndexDb = (sbMag > 0 && fundMag > 0)
        ? (float)(20.0 * log10(fundMag / sbMag)) : 99.0f;
    if      (faultIndexDb > 50) verdict = "HEALTHY (sidebands < -50 dB)";
    else if (faultIndexDb > 40) verdict = "WATCH (sidebands rising)";
    else                        verdict = "FAULT SIGNATURE - inspect rotor/bearings";

    if (logger) {
      logger("MCSA: fault index " + String(faultIndexDb, 0) +
             " dB -> " + verdict);
    }
    busy = false;
    return true;
  }

  // Magnitude of a single FFT bin from the last scan (for
  // diagnostics / future spectrum display). Bin 0 is DC.
  double binMagnitude(uint16_t bin) const {
    return (bin < FFT_N) ? vReal[bin] : 0.0;
  }

  // Frequency of the strongest NON-fundamental component in
  // the last scan - a quick pointer to where the spectrum is
  // dirty (slot harmonics, eccentricity components, inverter
  // residue) when the headline verdict needs a second look.
  // Bins within +/-3 of the fundamental are ignored.
  float spectrumPeakHz() const {
    int fundBin = (int)(fundamentalHz * FFT_N / SAMPLE_RATE);
    double best = 0;
    int bestBin = 0;
    for (int b = 2; b < (int)FFT_N / 2; b++) {
      if (abs(b - fundBin) <= 3) continue;
      if (vReal[b] > best) { best = vReal[b]; bestBin = b; }
    }
    return bestBin * SAMPLE_RATE / FFT_N;
  }

private:
  // Peak magnitude within +/-1 bin of `bin`, expressed in dB
  // BELOW the given fundamental magnitude (negative number).
  // Returns -99 dB when the bin is out of range or empty.
  double harmonicDb(int bin, double fundMag) const {
    if (fundMag <= 0 || bin < 1 || bin >= (int)FFT_N / 2 - 1)
      return -99.0;
    double peak = 0;
    for (int b = bin - 1; b <= bin + 1; b++)
      peak = max(peak, vReal[b]);
    return (peak > 0) ? 20.0 * log10(peak / fundMag) : -99.0;
  }
  double vReal[FFT_N];
  double vImag[FFT_N];
  ArduinoFFT<double> fft;
};
