// ============================================================
//  TORQUETWIN v2 - SENSORLESS FOC DRIVE + DIGITAL TWIN + MCSA
//  ESP32 + 3-Phase Gate Driver + Phase-Current Sensing
// ------------------------------------------------------------
//  Three advanced ideas in one drive:
//
//  1) SENSORLESS FIELD-ORIENTED CONTROL (FOC)
//     No encoder, no Hall sensors. Phase currents are measured,
//     transformed (Clarke -> Park) into the rotating d-q frame
//     and regulated by PI controllers; the inverse transform +
//     SVPWM drives the 3-phase bridge. Rotor position is
//     ESTIMATED by a Sliding Mode Observer (SMO) that tracks
//     the motor's back-EMF, with a tracking PLL cleaning up
//     the angle. Start-up is open-loop (I/f ramp) until the
//     back-EMF is strong enough for the observer to lock.
//
//  2) DIGITAL TWIN
//     A d-q model of the same PMSM runs in parallel on the
//     same voltage commands. If the real machine and its twin
//     disagree beyond a threshold for a sustained time,
//     something physical has changed - the drive flags a
//     twin-deviation warning BEFORE failure.
//
//  3) MOTOR CURRENT SIGNATURE ANALYSIS (MCSA)
//     At steady speed the firmware FFTs one phase current and
//     inspects the sidebands around the supply fundamental.
//     Sideband growth at (1 +/- 2s)f is the classic signature
//     of rotor/bearing problems - predictive maintenance from
//     a current sensor, no vibration hardware.
//
//  WHAT CHANGED IN v2
//    * Multi-file structure (below) instead of one monolith.
//    * Control loop runs from a HARDWARE TIMER flag at 5 kHz
//      (v1: 2 kHz micros() polling). Observer/filter constants
//      re-scaled for the new rate - see config.h.
//    * ADC current-offset AUTO-CALIBRATION at boot: the true
//      zero of each ACS712 channel is measured with the
//      driver disabled (v1 assumed exactly half supply).
//    * Dashboard also reports calibration status and SMO lock
//      quality; an overrun counter exposes missed control
//      ticks (e.g. during an MCSA scan).
//
//  MULTI-FILE STRUCTURE (all files live in this sketch folder)
//    config.h    - motor parameters, pin map, sensor scaling,
//                  PWM/timing constants, ALL gains/thresholds
//    foc.h       - PI controller, Clarke/Park transforms,
//                  voltage-circle limiter, SVPWM output
//    observer.h  - sliding-mode observer + tracking PLL
//    twin.h      - digital-twin model + deviation judgement
//    mcsa.h      - FFT current-signature analysis (arduinoFFT)
//    This file   - hardware bring-up, calibration, the 5 kHz
//                  control step, web dashboard and server.
//
//  Requires the "arduinoFFT" library ( Enrique Condes ).
//
//  SAFETY: this firmware drives real power hardware. Use a
//  current-limited bench supply, keep DC-bus voltage modest
//  (24 V class) while learning, and never bypass the driver's
//  hardware protection.
//
//  Author : Deepesh Chauhan, B.Tech Electrical Engineering
//           RGGEC Kangra
// ============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

#include "config.h"
#include "foc.h"
#include "observer.h"
#include "twin.h"
#include "mcsa.h"

// ---------------- WI-FI (replace with your credentials) ------
const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// ---------------- DRIVE STATE ----------------------------------
volatile bool driveEnabled = false;
float speedRefRPM   = DEFAULT_SPEED_RPM;   // dashboard set-point
float thetaEst      = 0.0f;                // observer angle (elec rad)
float omegaEst      = 0.0f;                // observer speed (elec rad/s)
float id = 0, iq = 0;                      // measured dq currents
float vd = 0, vq = 0;                      // PI outputs (pre-limit)
float vbus = 24.0f;
bool  observerLocked = false;
float startupRamp = 0.0f;                  // 0..1, mirrors ramp.fraction

String eventLog = "Boot\n";

// ---------------- MODULE INSTANCES ------------------------------
PI piId  = { PI_ID_KP,  PI_ID_KI,  0, PI_ID_OUTMAX  };
PI piIq  = { PI_IQ_KP,  PI_IQ_KI,  0, PI_IQ_OUTMAX  };
PI piSpd = { PI_SPD_KP, PI_SPD_KI, 0, PI_SPD_OUTMAX };  // out: iq ref

SMO          smo;          // observer.h
Twin         twin;         // twin.h
MCSAAnalyzer mcsa;         // mcsa.h
OpenLoopRamp ramp;         // foc.h - start-up I/f ramp state

// ---------------- EVENT LOG --------------------------------------
void logEvent(const String& msg) {
  eventLog += msg + "\n";
  if (eventLog.length() > 1600) eventLog = eventLog.substring(500);
  Serial.println(msg);
}

// ---------------- DRIVE ENABLE / DISABLE --------------------------
// Single point where the drive is switched, so the dashboard,
// the serial console and any future safety logic can never
// disagree about the gate-driver state or the log trail.
void setDrive(bool on) {
  driveEnabled = on;
  digitalWrite(DRV_EN_PIN, on ? HIGH : LOW);
  logEvent(on ? "Drive START (open-loop ramp -> sensorless FOC)"
              : "Drive STOP");
}

// =============================================================
//  CURRENT-OFFSET AUTO-CALIBRATION
//  Runs ONCE at boot with the gate driver disabled, so no phase
//  current can flow: the average ADC reading of each channel
//  is then the sensor's true zero-current output. v1 assumed
//  exactly VREF/2; real ACS712 zeros wander by tens of mV,
//  which at 185 mV/A is a few hundred mA of phantom current -
//  enough to bias the Park transform and upset the observer.
// =============================================================
float calOffsetVoltsA = ADC_VREF / 2.0f;   // pre-cal fallback = v1
float calOffsetVoltsB = ADC_VREF / 2.0f;
bool  calDone = false;

void calibrateCurrentOffsets() {
  digitalWrite(DRV_EN_PIN, LOW);           // driver hard-off
  pwmOff();
  double sumA = 0, sumB = 0;
  for (int i = 0; i < CAL_SAMPLES; i++) {  // 300 samples/channel
    sumA += analogRead(IA_PIN);
    sumB += analogRead(IB_PIN);
    delayMicroseconds(200);
  }
  float rawA = (float)(sumA / CAL_SAMPLES);
  float rawB = (float)(sumB / CAL_SAMPLES);
  calOffsetVoltsA = rawA / ADC_FULL_SCALE * ADC_VREF;
  calOffsetVoltsB = rawB / ADC_FULL_SCALE * ADC_VREF;
  calDone = true;
  logEvent("CAL: IA zero = " + String(calOffsetVoltsA, 3) + " V (" +
           String(rawA, 0) + " counts), IB zero = " +
           String(calOffsetVoltsB, 3) + " V (" + String(rawB, 0) +
           " counts)");
}

// =============================================================
//  SENSOR READINGS
// =============================================================
float readPhaseCurrent(int pin) {
  // Median of 3 samples rejects single-sample ADC spikes (v1).
  int a = analogRead(pin), b = analogRead(pin), c = analogRead(pin);
  int raw = max(min(a, b), min(max(a, b), c));
  float volts = (raw / ADC_FULL_SCALE) * ADC_VREF;
  // v2: subtract the MEASURED per-channel zero, not VREF/2.
  float offset = (pin == IA_PIN) ? calOffsetVoltsA : calOffsetVoltsB;
  return (volts - offset) / CUR_SENS_V_PER_A;
}

float readVbus() {
  int raw = analogRead(VBUS_PIN);
  return (raw / ADC_FULL_SCALE) * ADC_VREF * VBUS_DIV;
}

// =============================================================
//  CONTROL TIMER - 5 kHz hardware flag
//  The ISR does nothing but raise a flag (and count overruns);
//  the control step itself runs in loop() context. This is
//  the standard professional pattern: deterministic cadence
//  from hardware, no blocking work inside the ISR.
// =============================================================
hw_timer_t* ctrlTimer = NULL;
volatile bool     ctrlTick     = false;
volatile uint32_t ctrlOverruns = 0;

void IRAM_ATTR onCtrlTimer() {
  if (ctrlTick) ctrlOverruns++;            // previous step not run
  ctrlTick = true;
}

void startControlTimer() {
  ctrlTimer = timerBegin(0, 80, true);     // 80 MHz / 80 = 1 us tick
  timerAttachInterrupt(ctrlTimer, &onCtrlTimer, true);
  timerAlarmWrite(ctrlTimer, 1000000 / CTRL_RATE_HZ, true);  // 200 us
  timerAlarmEnable(ctrlTimer);
}

// =============================================================
//  MAIN FOC STEP (5 kHz, timer-flagged)
// =============================================================
void focStep() {
  vbus = readVbus();
  float ia = readPhaseCurrent(IA_PIN);
  float ib = readPhaseCurrent(IB_PIN);
  float ic = -(ia + ib);

  float iAl, iBe;
  clarke(ia, ib, ic, iAl, iBe);

  if (!driveEnabled) {
    pwmOff();
    piId.reset(); piIq.reset(); piSpd.reset();
    smo.reset();                           // no stale lock on restart
    ramp.reset();
    startupRamp = 0;
    return;
  }

  // --- start-up: open-loop I/f ramp blended into observer ----
  float omegaRefE = rpmToElecRadS(speedRefRPM);
  ramp.step(omegaRefE, CTRL_DT);
  startupRamp = ramp.fraction;

  float thetaUsed;
  if (smo.locked && startupRamp > HANDOVER_RAMP) {
    thetaEst = smo.theta;
    omegaEst = smo.omega;
    thetaUsed = thetaEst;
  } else {
    thetaUsed = ramp.theta;
    omegaEst = ramp.omegaNow(omegaRefE);
  }
  observerLocked = smo.locked;

  park(iAl, iBe, thetaUsed, id, iq);

  // --- speed loop ---------------------------------------------
  float rpmEst = elecRadSToRpm(omegaEst);
  float iqRef = piSpd.run(speedRefRPM - rpmEst, CTRL_DT);
  iqRef = constrain(iqRef, -M.ratedA, M.ratedA);
  float idRef = 0.0f;                        // no field weakening

  // --- current loops with d-q decoupling feed-forward ---------
  float vdFF, vqFF;
  dqDecoupling(omegaEst, id, iq, vdFF, vqFF);
  vd = piId.run(idRef - id, CTRL_DT) + vdFF;
  vq = piIq.run(iqRef - iq, CTRL_DT) + vqFF;

  // --- voltage circle limit + SVPWM ---------------------------
  limitVoltageVector(vd, vq, vbus);
  float vAl, vBe;
  invPark(vd, vq, thetaUsed, vAl, vBe);
  svpwm(vAl, vBe, vbus);

  // --- observer + twin fed with commanded voltages ------------
  smo.step(vAl, vBe, iAl, iBe, CTRL_DT);
  twin.step(vd, vq, omegaEst, id, iq, driveEnabled, CTRL_DT, logEvent);
}

// MCSA scan wrapper - keeps the v1 call sites (dashboard button
// and serial 'm') unchanged. The scan blocks ~0.5 s; control
// ticks missed meanwhile show up in ctrlOverruns.
void runMCSA() {
  if (!driveEnabled) return;
  mcsa.run(omegaEst, readPhaseCurrent, logEvent);
}

// =============================================================
//  SERIAL DIAGNOSTICS
//  's' prints a one-shot status block; 'c' re-runs the offset
//  calibration (refused while the drive is running - a "zero"
//  measured with current flowing would corrupt every reading
//  that depends on it).
// =============================================================
void printStatus() {
  Serial.println("---- TorqueTwin status --------------------------");
  Serial.print("drive       : "); Serial.println(driveEnabled ? "RUNNING" : "STOPPED");
  Serial.print("speed ref   : "); Serial.print(speedRefRPM, 0); Serial.println(" rpm");
  Serial.print("speed est   : "); Serial.print(elecRadSToRpm(omegaEst), 0); Serial.println(" rpm");
  Serial.print("id / iq     : "); Serial.print(id, 2); Serial.print(" A / ");
  Serial.print(iq, 2); Serial.println(" A");
  Serial.print("vd / vq     : "); Serial.print(vd, 2); Serial.print(" V / ");
  Serial.print(vq, 2); Serial.println(" V");
  Serial.print("bus         : "); Serial.print(vbus, 1); Serial.println(" V");
  Serial.print("observer    : "); Serial.print(smo.locked ? "LOCKED" : "OPEN-LOOP");
  Serial.print(" (quality "); Serial.print(smo.lockQuality, 0);
  Serial.print("%, |e| "); Serial.print(smo.emfMag, 2); Serial.println(" V)");
  Serial.print("calibration : "); Serial.print(calDone ? "DONE" : "ASSUMED");
  Serial.print(" (A "); Serial.print(calOffsetVoltsA, 3);
  Serial.print(" V, B "); Serial.print(calOffsetVoltsB, 3); Serial.println(" V)");
  Serial.print("twin dev    : "); Serial.print(twin.deviationPct, 1); Serial.println(" %");
  Serial.print("mcsa        : "); Serial.print(mcsa.faultIndexDb, 0);
  Serial.print(" dB - "); Serial.println(mcsa.verdict);
  Serial.print("tick overruns: "); Serial.println(ctrlOverruns);
  Serial.println("-------------------------------------------------");
}

// =============================================================
//  DASHBOARD
// =============================================================
WebServer server(80);

const char DASHBOARD[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>TorqueTwin - Sensorless FOC Drive</title>
<style>
 body{font-family:'Segoe UI',Arial;background:#0b1120;color:#e2e8f0;margin:0;padding:16px}
 h1{color:#38bdf8;text-align:center;margin:4px}
 .sub{text-align:center;color:#94a3b8;margin-bottom:14px}
 .wrap{max-width:980px;margin:auto}
 .cards{display:flex;gap:12px;flex-wrap:wrap;justify-content:center}
 .card{background:#1e293b;border-radius:12px;padding:14px 18px;min-width:140px;text-align:center}
 .card b{font-size:1.6em;color:#f8fafc}.card span{color:#94a3b8;font-size:.85em}
 .ok{color:#4ade80}.warn{color:#fbbf24}.bad{color:#f87171}
 input[type=range]{width:320px}
 button{background:#0284c7;color:#fff;border:0;border-radius:8px;padding:10px 22px;font-size:1em;cursor:pointer;margin:4px}
 button.stop{background:#b91c1c}
 canvas{background:#0f172a;border:1px solid #334155;border-radius:12px;width:100%}
 pre{background:#0f172a;border-radius:8px;padding:10px;white-space:pre-wrap;max-height:150px;overflow:auto}
</style></head><body><div class="wrap">
<h1>🌀 TorqueTwin — Sensorless FOC Drive</h1>
<div class="sub">Sliding-Mode Observer · Digital Twin · MCSA Predictive Maintenance · v2 (5 kHz, auto-cal)</div>
<div class="cards">
 <div class="card"><span>SPEED</span><br><b id="rpm">0</b><br><span>rpm</span></div>
 <div class="card"><span>i q (torque)</span><br><b id="iq">0</b><br><span>A</span></div>
 <div class="card"><span>i d</span><br><b id="id">0</b><br><span>A</span></div>
 <div class="card"><span>BUS</span><br><b id="vb">0</b><br><span>V</span></div>
 <div class="card"><span>OBSERVER</span><br><b id="obs">--</b><br><span>SMO + PLL</span></div>
 <div class="card"><span>LOCK QUALITY</span><br><b id="lockq">--</b><br><span id="ovr">PLL phase health</span></div>
 <div class="card"><span>ADC CAL</span><br><b id="cal">--</b><br><span id="calv">zero offsets</span></div>
 <div class="card"><span>TWIN DEVIATION</span><br><b id="dev">0%</b><br><span id="devmsg">model vs real</span></div>
 <div class="card"><span>MCSA</span><br><b id="mcsa">--</b><br><span id="mcsav">not analysed</span></div>
</div>
<p style="text-align:center">
 Set speed: <input type="range" id="spd" min="200" max="3000" value="800" step="50">
 <b id="spdv">800</b> rpm<br>
 <button onclick="cmd('start')">▶ START</button>
 <button class="stop" onclick="cmd('stop')">■ STOP</button>
 <button onclick="cmd('mcsa')">Run MCSA Scan</button>
</p>
<h3>Speed / Iq — live trend</h3>
<canvas id="tr" width="940" height="240"></canvas>
<h3>Event Log</h3><pre id="log"></pre>
<script>
let hist=[];
const spd=document.getElementById('spd');
spd.oninput=()=>{document.getElementById('spdv').textContent=spd.value;};
spd.onchange=()=>fetch('/cmd?c=speed&v='+spd.value);
function cmd(c){fetch('/cmd?c='+c);}
function draw(){
 const cv=document.getElementById('tr'),x=cv.getContext('2d');
 x.clearRect(0,0,cv.width,cv.height);
 x.strokeStyle='#334155';x.beginPath();x.moveTo(40,10);x.lineTo(40,210);x.lineTo(920,210);x.stroke();
 if(hist.length<2)return;
 const mx=3200;
 x.strokeStyle='#38bdf8';x.lineWidth=2;x.beginPath();
 hist.forEach((v,i)=>{const px=40+i*(880/119),py=210-(v/mx)*190;i?x.lineTo(px,py):x.moveTo(px,py);});
 x.stroke();
}
async function tick(){
 const d=await(await fetch('/data')).json();
 document.getElementById('rpm').textContent=d.rpm.toFixed(0);
 document.getElementById('iq').textContent=d.iq.toFixed(2);
 document.getElementById('id').textContent=d.id.toFixed(2);
 document.getElementById('vb').textContent=d.vbus.toFixed(1);
 const o=document.getElementById('obs');
 o.textContent=d.locked?'LOCKED':'OPEN-LOOP';o.className=d.locked?'ok':'warn';
 const lq=document.getElementById('lockq');
 lq.textContent=d.lockQ.toFixed(0)+'%';
 lq.className=d.lockQ>70?'ok':(d.lockQ>40?'warn':'bad');
 document.getElementById('ovr').textContent='PLL health · tick overruns '+d.ovr;
 const cl=document.getElementById('cal');
 cl.textContent=d.cal?'DONE':'ASSUMED';cl.className=d.cal?'ok':'warn';
 document.getElementById('calv').textContent=d.cal?
   ('A '+d.calA.toFixed(3)+' V · B '+d.calB.toFixed(3)+' V'):'mid-supply fallback';
 const dv=document.getElementById('dev');
 dv.textContent=d.dev.toFixed(0)+'%';dv.className=d.dev>25?'bad':(d.dev>12?'warn':'ok');
 document.getElementById('mcsa').textContent=d.mcsaDb?d.mcsaDb.toFixed(0)+' dB':'--';
 document.getElementById('mcsav').textContent=d.mcsaV;
 document.getElementById('log').textContent=d.log;
 hist.push(d.rpm);if(hist.length>120)hist.shift();draw();
}
setInterval(tick,500);tick();
</script></div></body></html>)HTML";

void handleRoot() { server.send_P(200, "text/html", DASHBOARD); }

void handleData() {
  float rpmEst = elecRadSToRpm(omegaEst);
  String j = "{";
  j += "\"rpm\":" + String(rpmEst, 1);
  j += ",\"iq\":" + String(iq, 2);
  j += ",\"id\":" + String(id, 2);
  j += ",\"vbus\":" + String(vbus, 1);
  j += ",\"locked\":" + String(observerLocked ? "true" : "false");
  j += ",\"lockQ\":" + String(smo.lockQuality, 0);
  j += ",\"cal\":" + String(calDone ? "true" : "false");
  j += ",\"calA\":" + String(calOffsetVoltsA, 3);
  j += ",\"calB\":" + String(calOffsetVoltsB, 3);
  j += ",\"dev\":" + String(twin.deviationPct, 1);
  j += ",\"mcsaDb\":" + String(mcsa.faultIndexDb, 1);
  j += ",\"mcsaV\":\"" + mcsa.verdict + "\"";
  j += ",\"ovr\":" + String(ctrlOverruns);
  j += ",\"log\":\"";
  String esc = eventLog; esc.replace("\n", "\\n"); esc.replace("\"", "'");
  j += esc + "\"}";
  server.send(200, "application/json", j);
}

void handleCmd() {
  String c = server.arg("c");
  if (c == "start") setDrive(true);
  if (c == "stop")  setDrive(false);
  if (c == "speed") { speedRefRPM = server.arg("v").toFloat(); logEvent("Speed ref = " + String(speedRefRPM, 0) + " rpm"); }
  if (c == "mcsa")  { server.send(200, "text/plain", "scan done"); runMCSA(); return; }
  server.send(200, "text/plain", "ok");
}

// =============================================================
//  SETUP & LOOP
// =============================================================
void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
  pinMode(DRV_EN_PIN, OUTPUT);
  digitalWrite(DRV_EN_PIN, LOW);

  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES_BITS); ledcAttachPin(PWM_A_PIN, PWM_CH_A);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES_BITS); ledcAttachPin(PWM_B_PIN, PWM_CH_B);
  ledcSetup(PWM_CH_C, PWM_FREQ, PWM_RES_BITS); ledcAttachPin(PWM_C_PIN, PWM_CH_C);
  pwmOff();

  // Boot calibration: driver is disabled and PWM is at zero,
  // so the current channels carry no real current - what is
  // measured here is each sensor's true zero offset.
  calibrateCurrentOffsets();

  // Echo the loaded motor model once, so every log/diagnostic
  // session records exactly which machine the observer and
  // the twin were tuned for.
  logEvent("Motor: Rs=" + String(M.Rs, 2) + " ohm, Ld=Lq=" +
           String(M.Ld * 1000.0f, 2) + " mH, flux=" +
           String(M.flux, 4) + " Wb, p=" + String(M.polePairs) +
           ", rated " + String(M.ratedA, 0) + " A");

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

  // Control timer starts LAST: from this point focStep() can
  // be requested at 5 kHz with every module initialised.
  startControlTimer();
  logEvent("TorqueTwin v2 ready (5 kHz timer FOC). Serial keys: g=start, x=stop, m=MCSA, s=status, c=recalibrate.");
}

void loop() {
  server.handleClient();

  if (ctrlTick) {                          // one 5 kHz tick due
    ctrlTick = false;
    focStep();
  }

  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'g') setDrive(true);
    if (ch == 'x') setDrive(false);
    if (ch == 'm') runMCSA();
    if (ch == 's') printStatus();
    if (ch == 'c') {
      if (!driveEnabled) calibrateCurrentOffsets();
      else logEvent("CAL refused: stop the drive before recalibrating");
    }
  }
}
