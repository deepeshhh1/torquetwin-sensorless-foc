// ============================================================
//  TORQUETWIN - SENSORLESS FOC DRIVE with DIGITAL TWIN + MCSA
//  ESP32 + 3-Phase Gate Driver (DRV8302 class) + ACS712
// ------------------------------------------------------------
//  Three advanced drive techniques in one firmware:
//
//  1) SENSORLESS FOC - Field-Oriented Control with NO encoder
//     and NO Hall sensors. Rotor position is estimated by a
//     SLIDING MODE OBSERVER (SMO) that recovers the back-EMF
//     from measured voltages/currents, followed by a PLL that
//     extracts a clean angle + speed.
//  2) DIGITAL TWIN - a real-time d-q model of the same motor
//     runs in parallel with the physical drive, fed with the
//     same voltage commands. Sustained model-vs-real deviation
//     flags demagnetisation, bearing drag or phase faults
//     BEFORE they become failures.
//  3) MCSA - Motor Current Signature Analysis: an FFT of the
//     phase current looks for the (1 +/- 2s)f sidebands that
//     betray rotor-bar / eccentricity faults.
//
//  LIBRARY: install "arduinoFFT" by Enrique Condes (v2.x) via
//  the Arduino Library Manager.
//
//  Author : Deepesh Chauhan, B.Tech Electrical Engineering
//           RGGEC Kangra
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>
#include <arduinoFFT.h>
#include <math.h>

// ---------------- WI-FI (replace with your credentials) -----
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ---------------- MOTOR PARAMETERS ---------------------------
// Measure for YOUR motor (see tuning-guide.md)
struct MotorParams {
  float Rs     = 0.35f;      // stator resistance, ohm/phase
  float Ld     = 0.00045f;   // d-axis inductance, H
  float Lq     = 0.00055f;   // q-axis inductance, H
  float flux   = 0.0065f;    // rotor flux linkage, Wb
  int   polePairs = 7;       // 14-pole outrunner example
  float J      = 0.00002f;   // inertia, kg m^2 (twin)
  float B      = 0.00001f;   // friction, N m s (twin)
};
MotorParams M;

// ---------------- PINS / PWM ---------------------------------
#define PWM_A_H 25
#define PWM_B_H 26
#define PWM_C_H 27
#define EN_GATE 14
#define CURR_A_PIN 34        // ACS712 phase A
#define CURR_B_PIN 35        // ACS712 phase B
#define VBUS_PIN   32

const int   PWM_FREQ = 20000;        // 20 kHz switching
const int   PWM_RES  = 10;           // bits -> max duty 1023
const float CTRL_DT  = 0.0005f;      // 2 kHz control loop
const float SENSE_VREF = 3.3f;
const float ACS_SENS   = 0.185f;     // V/A for ACS712-5A
const float VBUS_RATIO = 11.0f;      // divider ratio

// ---------------- CONTROLLERS --------------------------------
struct PI {
  float kp, ki, integ = 0, outMin, outMax;
  float run(float err, float dt) {
    integ += err * dt;
    float out = kp * err + ki * integ;
    if (out > outMax) { out = outMax; integ -= err * dt; }
    if (out < outMin) { out = outMin; integ -= err * dt; }
    return out;
  }
  void reset() { integ = 0; }
};
PI piId  {1.8f, 900.0f, 0, -12, 12};
PI piIq  {1.8f, 900.0f, 0, -12, 12};
PI piSpd {0.06f, 1.2f, 0, -3.0f, 3.0f};   // output = iq reference (A)

// ---------------- SLIDING MODE OBSERVER ----------------------
struct SMO {
  float iEstA = 0, iEstB = 0;      // estimated alpha-beta currents
  float emfA = 0, emfB = 0;         // filtered back-EMF (alpha-beta)
  float K = 9.0f;                   // sliding gain (V)
  float lpfK = 0.06f;               // EMF low-pass coefficient
  float theta = 0;                  // estimated angle (rad, elec)
  float omega = 0;                  // estimated speed (rad/s, elec)
  float pllKp = 60.0f, pllKi = 900.0f, pllInteg = 0;

  // one observer step; vAlpha/vBeta = applied stator voltage
  void step(float vA, float vB, float iA, float iB, float dt) {
    // current model: di/dt = (v - R i - e)/L
    float L = 0.5f * (M.Ld + M.Lq);
    float errA = iEstA - iA;
    float errB = iEstB - iB;
    float zA = (errA > 0) ? K : -K;      // sliding surface sign()
    float zB = (errB > 0) ? K : -K;
    iEstA += ((vA - M.Rs * iEstA - zA) / L) * dt;
    iEstB += ((vB - M.Rs * iEstB - zB) / L) * dt;
    // equivalent control -> back-EMF estimate (low-passed z)
    emfA += (zA - emfA) * lpfK;
    emfB += (zB - emfB) * lpfK;
    // PLL on EMF vector: for a PMSM, e is perpendicular to the
    // rotor flux, so angle = atan2(-eAlpha, eBeta)
    float rawTheta = atan2f(-emfA, emfB);
    float perr = sinf(rawTheta - theta);
    pllInteg += perr * dt;
    omega = pllKp * perr + pllKi * pllInteg;
    theta += omega * dt;
    if (theta > PI) theta -= 2 * PI;
    if (theta < -PI) theta += 2 * PI;
  }
  float emfMagnitude() { return sqrtf(emfA * emfA + emfB * emfB); }
};
SMO smo;
bool observerLocked = false;

// ---------------- FOC STATE ----------------------------------
float id = 0, iq = 0, vdCmd = 0, vqCmd = 0;
float speedRPM = 0, speedRefRPM = 0;
float vBus = 24.0f;
bool driveEnabled = false;
bool useObserver = false;              // false during start-up ramp
float rampTheta = 0, rampOmega = 0;    // open-loop start-up
String motorLog = "Boot\n";

// ---------------- DIGITAL TWIN --------------------------------
struct Twin {
  float id = 0, iq = 0, wm = 0, theta = 0;
  float deviation = 0;                   // % model-vs-real current
  bool warning = false;
  unsigned long devSince = 0;
  void step(float vd, float vq, float dt) {
    // electrical dynamics (same equations as the real machine)
    float we = wm * M.polePairs;
    float did = (vd - M.Rs * id + we * M.Lq * iq) / M.Ld;
    float diq = (vq - M.Rs * iq - we * M.Ld * id - we * M.flux) / M.Lq;
    id += did * dt; iq += diq * dt;
    // mechanical dynamics: Te = 1.5 p [flux iq + (Ld-Lq) id iq]
    float Te = 1.5f * M.polePairs * (M.flux * iq + (M.Ld - M.Lq) * id * iq);
    float Tl = M.B * wm;                // friction-only load model
    wm += ((Te - Tl) / M.J) * dt;
    theta += we * dt;
  }
  void compare(float idReal, float iqReal) {
    float mag = sqrtf(idReal * idReal + iqReal * iqReal);
    float dmag = sqrtf((id - idReal) * (id - idReal) +
                       (iq - iqReal) * (iq - iqReal));
    deviation = (mag > 0.3f) ? 100.0f * dmag / mag : 0;
    if (driveEnabled && deviation > 25.0f) {
      if (devSince == 0) devSince = millis();
      if (!warning && millis() - devSince > 5000) {
        warning = true;
        logEvent("TWIN WARNING: model-vs-real deviation " +
                 String(deviation, 0) + "% sustained - inspect motor");
      }
    } else { devSince = 0; warning = false; }
  }
};
Twin twin;

// ---------------- MCSA ----------------------------------------
const int FFT_N = 1024;
double fftReal[FFT_N], fftImag[FFT_N];
ArduinoFFT<double> FFT(fftReal, fftImag, FFT_N, 2000.0);
String mcsaVerdict = "NOT RUN";
float mcsaFaultIndex = 0;

void runMCSA() {
  if (!driveEnabled || fabsf(speedRPM) < 100) {
    mcsaVerdict = "NEED STEADY RUN >100rpm";
    return;
  }
  // capture 1024 samples of phase A at 2 kHz
  for (int i = 0; i < FFT_N; i++) {
    fftReal[i] = readPhaseCurrent(CURR_A_PIN);
    fftImag[i] = 0;
    delayMicroseconds(500);
  }
  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  double f1;
  FFT.garbageCheck();
  f1 = FFT.majorPeak();
  if (f1 < 5) { mcsaVerdict = "NO FUNDAMENTAL FOUND"; return; }
  int binFund = (int)(f1 * FFT_N / 2000.0);
  float fundMag = fftReal[binFund];
  // slip estimate from observer speed vs synchronous (stator)
  // frequency: sidebands sit at (1 +/- 2s) f1
  float s = 0.02f;                        // assume ~2% until measured
  int offsetBins = max(1, (int)(2 * s * f1 * FFT_N / 2000.0));
  float sb = 0;
  for (int k = -1; k <= 1; k++) {
    int lo = binFund - offsetBins + k, hi = binFund + offsetBins + k;
    if (lo > 1) sb = max(sb, (float)fftReal[lo]);
    if (hi < FFT_N / 2) sb = max(sb, (float)fftReal[hi]);
  }
  mcsaFaultIndex = (fundMag > 0) ? 20.0f * log10f(sb / fundMag) : -99;
  if      (mcsaFaultIndex < -45) mcsaVerdict = "HEALTHY";
  else if (mcsaFaultIndex < -35) mcsaVerdict = "WATCH";
  else                           mcsaVerdict = "FAULT SIGNATURE - INSPECT";
  logEvent("MCSA: f1=" + String(f1, 1) + "Hz sideband=" +
           String(mcsaFaultIndex, 1) + "dB -> " + mcsaVerdict);
}

// ---------------- SENSING -------------------------------------
float readPhaseCurrent(uint8_t pin) {
  int raw = analogRead(pin);
  float v = (raw / 4095.0f) * SENSE_VREF;
  return (v - SENSE_VREF / 2.0f) / ACS_SENS;
}
float readVbus() {
  int raw = analogRead(VBUS_PIN);
  return (raw / 4095.0f) * SENSE_VREF * VBUS_RATIO;
}

// ---------------- LOGGING --------------------------------------
void logEvent(const String& msg) {
  motorLog += msg + "\n";
  if (motorLog.length() > 1600) motorLog = motorLog.substring(500);
  Serial.println(msg);
}

// =============================================================
//  FOC CORE
// =============================================================
inline void clarke(float a, float b, float c, float& al, float& be) {
  al = a;
  be = (a + 2.0f * b) / 1.7320508f;
}
inline void park(float al, float be, float th, float& d, float& q) {
  float s = sinf(th), c = cosf(th);
  d = al * c + be * s;
  q = -al * s + be * c;
}
inline void invPark(float d, float q, float th, float& al, float& be) {
  float s = sinf(th), c = cosf(th);
  al = d * c - q * s;
  be = d * s + q * c;
}

// SVPWM via min-max (zero-sequence) injection, duties 0..1
void svpwm(float vAl, float vBe, float vdc) {
  float va = vAl;
  float vb = -0.5f * vAl + 0.8660254f * vBe;
  float vc = -0.5f * vAl - 0.8660254f * vBe;
  float vmax = max(va, max(vb, vc)), vmin = min(va, min(vb, vc));
  float voff = -(vmax + vmin) * 0.5f;
  int top = (1 << PWM_RES) - 1;
  ledcWrite(0, constrain((int)(((va + voff) / vdc + 0.5f) * top), 0, top));
  ledcWrite(1, constrain((int)(((vb + voff) / vdc + 0.5f) * top), 0, top));
  ledcWrite(2, constrain((int)(((vc + voff) / vdc + 0.5f) * top), 0, top));
}

void controlStep(float dt) {
  float ia = readPhaseCurrent(CURR_A_PIN);
  float ib = readPhaseCurrent(CURR_B_PIN);
  float ic = -(ia + ib);
  vBus = max(6.0f, readVbus());

  // angle source: open-loop ramp until the observer locks
  float thetaCtl;
  if (!useObserver) {
    rampOmega = (speedRefRPM * 2 * PI / 60.0f) * M.polePairs;
    rampTheta += rampOmega * 0.35f * dt;    // gentle accel (35%)
    if (rampTheta > PI) rampTheta -= 2 * PI;
    thetaCtl = rampTheta;
    if (smo.emfMagnitude() > 1.2f && fabsf(speedRefRPM) > 250) {
      useObserver = true;
      observerLocked = true;
      logEvent("Observer LOCKED - closed-loop sensorless");
    }
  } else {
    thetaCtl = smo.theta;
  }

  float al, be;
  clarke(ia, ib, ic, al, be);
  park(al, be, thetaCtl, id, iq);

  // speed loop (50 Hz) -> iq reference
  static int div = 0;
  static float iqRef = 0;
  float omegaMech = useObserver ? (smo.omega / M.polePairs) : rampOmega / M.polePairs;
  speedRPM = omegaMech * 60.0f / (2 * PI);
  if (++div >= 10) {
    div = 0;
    iqRef = piSpd.run(speedRefRPM - speedRPM, dt * 10);
  }

  // current loops with d-q decoupling feed-forward
  float we = omegaMech * M.polePairs;
  float vd = piId.run(0 - id, dt) - we * M.Lq * iq;
  float vq = piIq.run(iqRef - iq, dt) + we * M.Ld * id + we * M.flux;
  // voltage circle limit
  float vmag = sqrtf(vd * vd + vq * vq);
  float vmax = vBus * 0.577f;
  if (vmag > vmax) { vd *= vmax / vmag; vq *= vmax / vmag; }
  vdCmd = vd; vqCmd = vq;

  float vAl, vBe;
  invPark(vd, vq, thetaCtl, vAl, vBe);
  if (driveEnabled) svpwm(vAl, vBe, vBus);
  else { ledcWrite(0, 0); ledcWrite(1, 0); ledcWrite(2, 0); }

  // observer runs on the APPLIED alpha-beta voltage
  smo.step(vAl, vBe, al, be, dt);
  if (observerLocked && smo.emfMagnitude() < 0.4f && driveEnabled) {
    observerLocked = false;
    useObserver = false;
    logEvent("Observer lost lock - back to open-loop ramp");
  }

  // digital twin (same commands, parallel model)
  twin.step(vd, vq, dt);
  twin.compare(id, iq);
}

// =============================================================
//  DASHBOARD
// =============================================================
WebServer server(80);
const char DASHBOARD[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>TorqueTwin - Sensorless FOC</title>
<style>
 body{font-family:'Segoe UI',Arial;background:#0b1120;color:#e2e8f0;margin:0;padding:16px}
 h1{color:#a78bfa;text-align:center;margin:4px}
 .sub{text-align:center;color:#94a3b8;margin-bottom:14px}
 .wrap{max-width:980px;margin:auto}
 .cards{display:flex;gap:12px;flex-wrap:wrap;justify-content:center}
 .card{background:#1e293b;border-radius:12px;padding:14px 18px;min-width:150px;text-align:center}
 .card b{font-size:1.5em;color:#f8fafc}.card span{color:#94a3b8;font-size:.85em}
 button{background:#7c3aed;color:#fff;border:0;border-radius:8px;padding:10px 18px;font-size:1em;cursor:pointer;margin:4px}
 button:hover{background:#6d28d9}
 input[type=range]{width:280px}
 canvas{background:#0f172a;border:1px solid #334155;border-radius:12px;width:100%}
 pre{background:#0f172a;border-radius:8px;padding:10px;white-space:pre-wrap;max-height:150px;overflow:auto}
</style></head><body><div class="wrap">
<h1>\U0001F300 TorqueTwin — Sensorless FOC Drive</h1>
<div class="sub">Sliding-Mode Observer · Digital Twin · MCSA Predictive Maintenance</div>
<div class="cards">
 <div class="card"><span>SPEED</span><br><b id="rpm">--</b><br><span>rpm</span></div>
 <div class="card"><span>i d / i q</span><br><b id="idiq">--</b><br><span>amps</span></div>
 <div class="card"><span>DC BUS</span><br><b id="vbus">--</b><br><span>volts</span></div>
 <div class="card"><span>OBSERVER</span><br><b id="obs">--</b><br><span>SMO + PLL</span></div>
 <div class="card"><span>TWIN DEVIATION</span><br><b id="dev">--</b><br><span>% model vs real</span></div>
 <div class="card"><span>MCSA</span><br><b id="mcsa">--</b><br><span>health verdict</span></div>
</div>
<p style="text-align:center">
 Speed reference: <input type="range" id="spd" min="0" max="2500" value="0" step="50"
   oninput="document.getElementById('spdv').textContent=this.value">
 <span id="spdv">0</span> rpm
 <button onclick="setRef()">SET SPEED</button>
 <button onclick="cmd('start')">▶ START</button>
 <button onclick="cmd('stop')">■ STOP</button>
 <button onclick="cmd('mcsa')">Run MCSA</button>
</p>
<h3>Speed Trend</h3>
<canvas id="cv" width="940" height="220"></canvas>
<h3>Event Log</h3><pre id="log"></pre>
<script>
let hist=[];
function cmd(c){fetch('/cmd?c='+c);}
function setRef(){fetch('/cmd?c=ref&v='+document.getElementById('spd').value);}
function draw(){
 const cv=document.getElementById('cv'),x=cv.getContext('2d');
 x.clearRect(0,0,cv.width,cv.height);
 x.strokeStyle='#a78bfa';x.lineWidth=2;x.beginPath();
 hist.forEach((v,i)=>{const px=i*(cv.width/119),py=cv.height-10-(v/2800)*(cv.height-20);
  i?x.lineTo(px,py):x.moveTo(px,py);});
 x.stroke();
}
async function tick(){
 const d=await(await fetch('/data')).json();
 document.getElementById('rpm').textContent=d.rpm.toFixed(0);
 document.getElementById('idiq').textContent=d.id.toFixed(2)+' / '+d.iq.toFixed(2);
 document.getElementById('vbus').textContent=d.vbus.toFixed(1);
 document.getElementById('obs').textContent=d.observer;
 document.getElementById('dev').textContent=d.twinDev.toFixed(1);
 document.getElementById('mcsa').textContent=d.mcsa;
 document.getElementById('log').textContent=d.log;
 hist.push(d.rpm);if(hist.length>120)hist.shift();
 draw();
}
setInterval(tick,1000);tick();
</script></div></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", DASHBOARD); }

void handleData() {
  String j = "{";
  j += "\"rpm\":" + String(speedRPM, 0);
  j += ",\"id\":" + String(id, 2);
  j += ",\"iq\":" + String(iq, 2);
  j += ",\"vbus\":" + String(vBus, 1);
  j += ",\"observer\":\"" + String(observerLocked ? "LOCKED" : "OPEN-LOOP") + "\"";
  j += ",\"twinDev\":" + String(twin.deviation, 1);
  j += ",\"twinWarn\":" + String(twin.warning ? "true" : "false");
  j += ",\"mcsa\":\"" + mcsaVerdict + "\"";
  j += ",\"mcsaDb\":" + String(mcsaFaultIndex, 1);
  j += ",\"on\":" + String(driveEnabled ? "true" : "false");
  j += ",\"log\":\"";
  String esc = motorLog; esc.replace("\n", "\\n"); esc.replace("\"", "'");
  j += esc + "\"}";
  server.send(200, "application/json", j);
}

void handleCmd() {
  String c = server.arg("c");
  if (c == "start") {
    driveEnabled = true;
    digitalWrite(EN_GATE, HIGH);
    logEvent("Drive START (ref " + String(speedRefRPM, 0) + " rpm)");
  } else if (c == "stop") {
    driveEnabled = false;
    digitalWrite(EN_GATE, LOW);
    piId.reset(); piIq.reset(); piSpd.reset();
    useObserver = false; observerLocked = false;
    logEvent("Drive STOP");
  } else if (c == "ref") {
    speedRefRPM = server.arg("v").toFloat();
    logEvent("Speed ref = " + String(speedRefRPM, 0) + " rpm");
  } else if (c == "mcsa") {
    server.send(200, "text/plain", "mcsa running");
    runMCSA();
    return;
  }
  server.send(200, "text/plain", "ok");
}

// =============================================================
//  SETUP & LOOP
// =============================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  ledcSetup(0, PWM_FREQ, PWM_RES); ledcAttachPin(PWM_A_H, 0);
  ledcSetup(1, PWM_FREQ, PWM_RES); ledcAttachPin(PWM_B_H, 1);
  ledcSetup(2, PWM_FREQ, PWM_RES); ledcAttachPin(PWM_C_H, 2);
  pinMode(EN_GATE, OUTPUT); digitalWrite(EN_GATE, LOW);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 8000) delay(300);
  if (WiFi.status() == WL_CONNECTED) {
    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/cmd", handleCmd);
    server.begin();
    logEvent("Dashboard: http://" + WiFi.localIP().toString());
  }
  logEvent("TorqueTwin ready. Set speed, then START.");
}

void loop() {
  server.handleClient();
  static unsigned long lastCtrl = 0;
  unsigned long now = micros();
  if (now - lastCtrl >= (unsigned long)(CTRL_DT * 1e6f)) {
    lastCtrl = now;
    if (driveEnabled || true) controlStep(CTRL_DT);
  }
}
