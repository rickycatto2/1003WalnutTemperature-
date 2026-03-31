// =============================================================
//  STEPPER CLOCK — 4-MOTOR PRODUCTION SKETCH
//  ESP32 WROOM-32 + 4× TB6600 (PUL+/DIR+ tied to 3.3V → active-LOW)
//  4× STEPPERONLINE Nema 17 | 1600 steps/rev (1/8 microstep) | 2A | 59Ncm
//
//  WiFi AP:  SSID = "TempClock"   Pass = "tempclock"
//  Web UI:   http://192.168.4.1
//
//  Dial range: −25°F to +135°F = 160°F span
//  Steps:      1600 steps per full rotation (1/8 microstep)
//  Resolution: STEPS_PER_DEG = 1600 / 160 = 10.0
//  Deadband:   0.8°F  (= 2 steps)
//  Speed:      15 sec/rev → STEP_TICKS = 935 (ISR ticks between steps)
//              ISR fires every 10µs — pulse width = exactly 10µs
//
//  Behaviour:
//    • On boot: all 4 motors home CW to their limit switches (+135°F end)
//    • Every 2 s: read DS18B20
//    • Move all dials only if temperature changed > DEADBAND_F
//    • Once per day: re-home all motors then return to temp
//    • Web UI: per-motor trim (±steps), re-home all or individual
//
//  ── PIN ASSIGNMENTS ─────────────────────────────────────────
//  Motor 1: PUL=25  DIR=26  LIMIT=27   (Side A)
//  Motor 2: PUL=32  DIR=33  LIMIT=14   (Side A)
//  Motor 3: PUL=22  DIR=23  LIMIT=21   (Side B)
//  Motor 4: PUL=16  DIR=17  LIMIT=18   (Side B)
//  DS18B20: DATA=4                      (Side B)
//
//  NOTE: P12/P13 avoided — P12 is a boot-strap pin on ESP32.
//  P34/P35 avoided — input-only, no internal pull-up, floats at boot.
//
//  ── TB6600 WIRING NOTE ──────────────────────────────────────
//  DIR+ and PUL+ are tied to 3.3V on all drivers.
//  ESP32 controls DIR− and PUL−.
//  Active-LOW: LOW = asserted, HIGH = idle.
//  DIR_CW  = LOW  (DIR− pulled low → CW)
//  DIR_CCW = HIGH (DIR− high → CCW)
//  A step fires on the falling edge of PUL−.
//
//  ── TB6600 DIP SWITCH SETTINGS ──────────────────────────────
//  Microstep: 1/8 → SW1=ON  SW2=OFF SW3=ON
//  Current:  2.5A → SW4=OFF SW5=OFF SW6=ON
//  1600 steps/rev at 1/8 microstep = full 160°F dial sweep
//
//  ── STEPPER ARCHITECTURE ────────────────────────────────────
//  Stepping is driven by a hardware timer ISR (Timer 0, 80MHz/80=1MHz).
//  The ISR fires every ISR_TICK_US (10µs) and uses STEP_TICKS to count
//  motors one step if needed. This is completely independent of loop()
//  so WiFi, web server, and temperature reads cannot cause stutter.
// =============================================================

#include <WiFi.h>
#include <WebServer.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>

// ── ★ CALIBRATION CONSTANTS ★ ────────────────────────────────
static const float    STEPS_PER_DEG           = 10.0f;  // 1600 steps / 160°F
static const float    DIAL_MIN_F              = -25.0f;
static const float    DIAL_MAX_F              = 135.0f;
static const float    DEADBAND_F              =  0.8f;   // 8 steps
static const uint32_t DAILY_HOME_INTERVAL_MS  = 24UL * 60UL * 60UL * 1000UL;

// ── SPEED ─────────────────────────────────────────────────────
// 15 sec/rev = 15,000,000µs / 400 steps = 37,500µs per step.
// Increase to go slower, decrease to go faster.
// ISR tick rate — do not change
#define ISR_TICK_US       10UL      // ISR fires every 10µs

// Steps per second = 1,000,000 / (ISR_TICK_US * STEP_TICKS)
// 15 sec/rev @ 1600 steps = 1600/15 = ~107 steps/sec
// STEP_TICKS = 1,000,000 / (107 * 10) = ~935
// Increase STEP_TICKS to go slower, decrease to go faster.
#define STEP_TICKS        935UL     // ~15 sec/rev at 1600 steps

// ── MOTOR COUNT ───────────────────────────────────────────────
#define NUM_MOTORS 4

// ── PIN ASSIGNMENTS ───────────────────────────────────────────
const uint8_t PIN_PUL[NUM_MOTORS]   = { 25, 32, 22, 16 };
const uint8_t PIN_DIR[NUM_MOTORS]   = { 26, 33, 23, 17 };
const uint8_t PIN_LIMIT[NUM_MOTORS] = { 27, 14, 21, 18 };
#define PIN_TEMP 4

// ── ACTIVE-LOW LOGIC (PUL+/DIR+ tied to 3.3V) ────────────────
#define PUL_ASSERT  LOW    // falling edge fires a step
#define PUL_IDLE    HIGH
#define DIR_CW      LOW    // DIR− low  → CW
#define DIR_CCW     HIGH   // DIR− high → CCW
#define DIR_SETUP_US 10

// ── STATE MACHINE ─────────────────────────────────────────────
enum SystemState    { SYS_HOMING, SYS_MOVING, SYS_IDLE };
enum PostHomeAction { POST_HOME_GO_TO_TEMP, POST_HOME_NONE };

// ── PER-MOTOR STATE ───────────────────────────────────────────
// ISR owns: state, currentSteps, targetSteps, directionCW,
//           ticksRemaining, pulseActive, homingDone.
// loop() owns: postHomeAction, trimSteps.
// Shared fields are volatile. loop() uses portENTER_CRITICAL when writing.
struct Motor {
  volatile SystemState   state          = SYS_HOMING;
  volatile long          currentSteps   = 0;
  volatile long          targetSteps    = 0;
  volatile bool          directionCW    = false;
  volatile bool          pulseActive    = false;  // true for one 10µs tick
  volatile uint32_t      ticksRemaining = 0;      // ISR ticks until next step
  volatile bool          homingDone     = false;
           PostHomeAction postHomeAction = POST_HOME_GO_TO_TEMP;
           int            trimSteps     = 0;
};

Motor motors[NUM_MOTORS];

// ── DIRECTION CONFIG (toggled from web UI) ───────────────────
// homeDirCW:    true  = home by spinning CW
//               false = home by spinning CCW
// hotMoreSteps: true  = higher temp = more steps from home
//               false = higher temp = fewer steps (closer to home)
volatile bool homeDirCW     = true;   // runtime value — loaded from NVS on boot
volatile bool hotMoreSteps  = false;  // runtime value — loaded from NVS on boot
Preferences   prefs;                  // NVS key-value store

// ── HARDWARE TIMER ────────────────────────────────────────────
hw_timer_t* stepTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ISR: fires every ISR_TICK_US (10µs). Owns all stepper pin writes.
//
// Per motor, each step looks like this across two consecutive ticks:
//   Tick N:   ticksRemaining hits 0 → assert PUL− LOW, set pulseActive
//   Tick N+1: pulseActive true → release PUL− HIGH (10µs pulse width),
//             commit step count, reload ticksRemaining for next step.
//
// This gives a clean 10µs pulse and perfectly even inter-step spacing
// regardless of what loop() is doing.
void IRAM_ATTR onStepTimer() {
  portENTER_CRITICAL_ISR(&timerMux);

  for (int i = 0; i < NUM_MOTORS; i++) {
    Motor& m = motors[i];

    if (m.state == SYS_IDLE) continue;

    if (m.pulseActive) {
      // ── Tick N+1: end pulse ──────────────────────────────────
      digitalWrite(PIN_PUL[i], PUL_IDLE);
      m.pulseActive = false;

      // Commit the step
      if (m.directionCW) m.currentSteps++;
      else               m.currentSteps--;

      // Check end conditions
      if (m.state == SYS_HOMING) {
        if (digitalRead(PIN_LIMIT[i]) == LOW) {
          m.currentSteps = 0;
          m.targetSteps  = 0;
          m.state        = SYS_IDLE;
          m.homingDone   = true;
          continue;
        }
      } else if (m.state == SYS_MOVING) {
        if (m.currentSteps == m.targetSteps) {
          m.state = SYS_IDLE;
          continue;
        }
      }

      // Reload counter for next step
      m.ticksRemaining = STEP_TICKS - 1;  // -1 because this tick counts

    } else {
      // ── Tick N: count down ───────────────────────────────────
      if (m.ticksRemaining > 0) {
        m.ticksRemaining--;
        continue;
      }

      // Counter expired — time to step. Check limits first.
      if (m.state == SYS_HOMING && digitalRead(PIN_LIMIT[i]) == LOW) {
        m.currentSteps = 0;
        m.targetSteps  = 0;
        m.state        = SYS_IDLE;
        m.homingDone   = true;
        continue;
      }
      if (m.state == SYS_MOVING && m.currentSteps == m.targetSteps) {
        m.state = SYS_IDLE;
        continue;
      }
      if (m.directionCW == homeDirCW && digitalRead(PIN_LIMIT[i]) == LOW) {
        // Safety: over-travel guard toward limit switch
        m.currentSteps = 0;
        m.targetSteps  = 0;
        m.state        = SYS_IDLE;
        continue;
      }

      // Assert pulse — released next tick
      digitalWrite(PIN_PUL[i], PUL_ASSERT);
      m.pulseActive = true;
    }
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}

// ── TEMPERATURE ───────────────────────────────────────────────
OneWire           ow(PIN_TEMP);
DallasTemperature sensors(&ow);
float    currentTempF   = -999.0f;
float    displayedTempF = -999.0f;
float    lastRawTempF   = -999.0f;
uint32_t lastTempMs     = 0;
#define TEMP_INTERVAL_MS  2000
#define TEMP_VALID_MIN_F -40.0f
#define TEMP_VALID_MAX_F 160.0f

inline bool tempIsValid(float t) {
  return (t != DEVICE_DISCONNECTED_F) &&
         (t >= TEMP_VALID_MIN_F) &&
         (t <= TEMP_VALID_MAX_F);
}

// ── DAILY HOMING ──────────────────────────────────────────────
uint32_t lastHomeMs = 0;

// ── WIFI / WEB ────────────────────────────────────────────────
const char* AP_SSID = "Infinity Signs - (816) 252 3337";
const char* AP_PASS = "*****";
WebServer server(80);

// ── WEB UI HTML ───────────────────────────────────────────────
// Split into 3KB chunks to avoid Arduino IDE raw-string truncation bug.

static const char PAGE_HTML_0[] PROGMEM = "<!DOCTYPE html>\n"
  "<html lang=\"en\">\n"
  "<head>\n"
  "<meta charset=\"utf-8\">\n"
  "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1,maximum-scale=1\">\n"
  "<title>Infinity Signs</title>\n"
  "<style>\n"
  ":root{\n"
  "  --bg:#1a1612;--surface:#232018;--border:#3a3328;\n"
  "  --amber:#e8a020;--amber-dim:#7a5010;\n"
  "  --green:#4a8a50;--text:#d8c8a8;--muted:#7a6a50;\n"
  "  --red:#8a2020;--r:10px;\n"
  "}\n"
  "*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;}\n"
  "body{background:var(--bg);color:var(--text);font-family:system-ui,-apple-system,sans-serif;font-size:15px;padding:16px 12px 60px;max-width:520px;margin:0 auto;}\n"
  "h1{font-size:22px;font-weight:700;color:var(--amber);letter-spacing:.03em;margin-bottom:2px;}\n"
  ".subtitle{color:var(--muted);font-size:12px;margin-bottom:16px;}\n"
  ".temp-hero{background:var(--surface);border:1px solid var(--border);border-radius:var(--r);padding:18px 20px;margin-bottom:12px;display:flex;align-items:center;gap:16px;}\n"
  ".temp-big{font-size:72px;font-weight:700;line-height:1;color:var(--amber);letter-spacing:-.02em;white-space:nowrap;}\n"
  ".temp-meta{color:var(--muted);font-size:12px;line-height:2;}\n"
  ".motor-grid{display:flex;flex-direction:column;gap:10px;margin-bottom:12px;}\n"
  ".motor-card{background:var(--surface);border:1px solid var(--border);border-radius:var(--r);padding:14px 16px;}\n"
  ".motor-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;}\n"
  ".motor-title{font-size:16px;font-weight:600;color:var(--amber);}\n"
  ".state-badge{font-size:11px;font-weight:600;padding:3px 10px;border-radius:20px;background:var(--amber-dim);color:var(--amber);text-transform:uppercase;letter-spacing:.04em;}\n"
  ".state-badge.idle  {background:#1a3a1a;color:var(--green);}\n"
  ".state-badge.moving{background:#3a2a10;color:#e8a020;}\n"
  ".state-badge.homing{background:#3a1a10;color:#e06040;}\n"
  ".motor-stats{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:4px;margin-bottom:14px;}\n"
  ".stat{background:var(--bg);border-radius:6px;padding:8px 6px;text-align:center;}\n"
  ".stat-label{font-size:10px;color:var(--muted);margin-bottom:3px;}\n"
  ".stat-val{font-size:14px;font-weight:600;color:var(--text);}\n"
  ".trim-row{display:flex;align-items:center;gap:6px;margin-bottom:8px;}\n"
  ".trim-label{font-size:12px;color:var(--muted);width:36px;flex-shrink:0;}\n"
  ".trim-btn{flex:1;height:44px;background:var(--border);border:1px solid #4a3a28;color:var(--text);font-size:16px;font-weight:600;border-radius:6px;cursor:pointer;display:flex;align-items:center;justify-content:center;transition:background .1s;user-select:none;}\n"
  ".trim-btn:active{background:#5a4a38;}\n"
  ".trim-val{font-size:15px;font-weight:700;color:var(--amber);min-width:44px;text-align:center;}\n"
  ".btn-row{display:flex;gap:6px;margin-top:4px;}\n"
  ".home-btn{flex:1;height:40px;background:transparent;border:1px solid var(--amber-dim);color:var(--amber);font-size:13px;font-weight:600;border-radius:6px;cursor:pointer;transition:background .1s;}\n"
  ".home-btn:active{background:var(--amber-dim);}\n"
  "";

static const char PAGE_HTML_1[] PROGMEM = ".reset-btn{height:40px;padding:0 12px;background:transparent;border:1px solid var(--red);color:#e06060;font-size:12px;font-weight:600;border-radius:6px;cursor:pointer;transition:background .1s;white-space:nowrap;}\n"
  ".reset-btn:active{background:var(--red);}\n"
  ".card{background:var(--surface);border:1px solid var(--border);border-radius:var(--r);padding:16px;margin-bottom:12px;}\n"
  ".card-title{font-size:14px;font-weight:600;color:var(--muted);text-transform:uppercase;letter-spacing:.06em;margin-bottom:12px;}\n"
  ".btn-home-all{width:100%;height:52px;background:var(--amber);color:#1a1612;border:none;font-size:15px;font-weight:700;border-radius:8px;cursor:pointer;letter-spacing:.02em;transition:opacity .1s;margin-bottom:8px;}\n"
  ".btn-home-all:active{opacity:.8;}\n"
  ".btn-secondary{width:100%;height:42px;background:transparent;border:1px solid var(--border);color:var(--text);font-size:14px;border-radius:8px;cursor:pointer;transition:background .1s;margin-bottom:8px;}\n"
  ".btn-secondary:active{background:var(--border);}\n"
  ".btn-danger{width:100%;height:42px;background:transparent;border:1px solid var(--red);color:#e06060;font-size:14px;border-radius:8px;cursor:pointer;transition:background .1s;}\n"
  ".btn-danger:active{background:var(--red);}\n"
  ".last-update{color:var(--muted);font-size:11px;margin-top:10px;text-align:center;}\n"
  ".toggle-row{display:flex;align-items:center;justify-content:space-between;padding:10px 0;border-bottom:1px solid var(--border);}\n"
  ".toggle-row:last-child{border-bottom:none;padding-bottom:0;}\n"
  ".toggle-label{font-size:14px;color:var(--text);}\n"
  ".toggle-sub{font-size:11px;color:var(--muted);margin-top:2px;}\n"
  ".toggle{position:relative;width:48px;height:26px;flex-shrink:0;}\n"
  ".toggle input{opacity:0;width:0;height:0;}\n"
  ".toggle-slider{position:absolute;inset:0;background:#3a3328;border-radius:26px;cursor:pointer;transition:background .2s;}\n"
  ".toggle-slider:before{content:'';position:absolute;width:20px;height:20px;left:3px;top:3px;background:#7a6a50;border-radius:50%;transition:transform .2s,background .2s;}\n"
  ".toggle input:checked + .toggle-slider{background:var(--amber-dim);}\n"
  ".toggle input:checked + .toggle-slider:before{transform:translateX(22px);background:var(--amber);}\n"
  "</style>\n"
  "</head>\n"
  "<body>\n"
  "<h1>Infinity Signs</h1>\n"
  "<div class=\"subtitle\">(816) 252-3337 &nbsp;·&nbsp; 4-dial vintage gauge</div>\n"
  "<div class=\"temp-hero\">\n"
  "  <div class=\"temp-big\" id=\"temp-big\">—</div>\n"
  "  <div class=\"temp-meta\" id=\"temp-meta\">Waiting…</div>\n"
  "</div>\n"
  "<div class=\"motor-grid\" id=\"motor-grid\"></div>\n"
  "\n"
  "<div class=\"card\">\n"
  "  <div class=\"card-title\">Direction Settings</div>\n"
  "  <div class=\"toggle-row\">\n"
  "    <div>\n"
  "      <div class=\"toggle-label\">Home direction</div>\n"
  "      <div class=\"toggle-sub\" id=\"home-dir-label\">CW</div>\n"
  "    </div>\n"
  "    <label class=\"toggle\">\n"
  "      <input type=\"checkbox\" id=\"tog-home\" onchange=\"setConfig()\">\n"
  "      <span class=\"toggle-slider\"></span>\n"
  "    </label>\n"
  "  </div>\n"
  "  <div class=\"toggle-row\">\n"
  "    <div>\n"
  "      <div class=\"toggle-label\">Hotter = more steps</div>\n"
  "";

static const char PAGE_HTML_2[] PROGMEM = "      <div class=\"toggle-sub\" id=\"hot-dir-label\">OFF</div>\n"
  "    </div>\n"
  "    <label class=\"toggle\">\n"
  "      <input type=\"checkbox\" id=\"tog-hot\" onchange=\"setConfig()\">\n"
  "      <span class=\"toggle-slider\"></span>\n"
  "    </label>\n"
  "  </div>\n"
  "</div>\n"
  "\n"
  "<div class=\"card\">\n"
  "  <div class=\"card-title\">Global Controls</div>\n"
  "  <button class=\"btn-home-all\" onclick=\"homeAll()\">↤ Re-home All Motors</button>\n"
  "  <button class=\"btn-secondary\" onclick=\"poll()\">↻ Refresh</button>\n"
  "  <button class=\"btn-danger\" onclick=\"resetAllTrims()\">✕ Reset All Trims to Zero</button>\n"
  "  <div class=\"last-update\" id=\"last-update\"></div>\n"
  "</div>\n"
  "\n"
  "<script>\n"
  "const NAMES=['North','East','South','West'];\n"
  "function post(u,b){return fetch(u,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b||''});}\n"
  "function homeAll(){post('/home_all').catch(()=>{});}\n"
  "function homeMotor(i){post('/home_motor','motor='+i).catch(()=>{});}\n"
  "function trimMotor(i,d){post('/trim_motor','motor='+i+'&delta='+d).catch(()=>{});}\n"
  "function resetTrim(i){if(confirm('Reset '+NAMES[i]+' trim to zero?'))post('/reset_trim','motor='+i).catch(()=>{});}\n"
  "function resetAllTrims(){if(confirm('Reset ALL trims to zero?'))post('/reset_all_trims').catch(()=>{});}\n"
  "function setConfig(){\n"
  "  const homeCW=document.getElementById('tog-home').checked;\n"
  "  const hotMoreSteps=document.getElementById('tog-hot').checked;\n"
  "  document.getElementById('home-dir-label').textContent=homeCW?'CW':'CCW';\n"
  "  document.getElementById('hot-dir-label').textContent=hotMoreSteps?'ON':'OFF';\n"
  "  post('/set_config','homeCW='+(homeCW?1:0)+'&hotMoreSteps='+(hotMoreSteps?1:0)).catch(()=>{});\n"
  "}\n"
  "function badge(s){const c=s==='Idle'?'idle':s==='Moving'?'moving':'homing';return '<span class=\"state-badge '+c+'\">'+s+'</span>';}\n"
  "function sign(n){return n>=0?'+':'';}\n"
  "function buildGrid(d){\n"
  "  const g=document.getElementById('motor-grid');\n"
  "  if(g.children.length===0){for(let i=0;i<4;i++){const c=document.createElement('div');c.className='motor-card';c.id='mc'+i;g.appendChild(c);}}\n"
  "  for(let i=0;i<4;i++){\n"
  "    const m=d.motors[i];\n"
  "    document.getElementById('mc'+i).innerHTML=\n"
  "      '<div class=\"motor-header\"><div class=\"motor-title\">'+NAMES[i]+'</div>'+badge(m.state)+'</div>'+\n"
  "      '<div class=\"motor-stats\">'+\n"
  "        '<div class=\"stat\"><div class=\"stat-label\">Position</div><div class=\"stat-val\">'+m.posDegF.toFixed(1)+'°</div></div>'+\n"
  "        '<div class=\"stat\"><div class=\"stat-label\">Target</div><div class=\"stat-val\">'+m.targetDegF.toFixed(1)+'°</div></div>'+\n"
  "        '<div class=\"stat\"><div class=\"stat-label\">Steps</div><div class=\"stat-val\">'+m.steps+'</div></div>'+\n"
  "        '<div class=\"stat\"><div class=\"stat-label\">Trim</div><div class=\"stat-val\">'+sign(m.trim)+m.trim+'</div></div>'+\n"
  "      '</div>'+\n"
  "      '<div class=\"trim-row\">'+\n"
  "        '<div class=\"trim-label\">Trim</div>'+\n"
  "        '<button class=\"trim-btn\" onclick=\"trimMotor('+i+',-5)\">−5</button>'+\n"
  "        '<button class=\"trim-btn\" onclick=\"trimMotor('+i+',-1)\">−1</button>'+\n"
  "";

static const char PAGE_HTML_3[] PROGMEM = "        '<div class=\"trim-val\">'+sign(m.trim)+m.trim+'</div>'+\n"
  "        '<button class=\"trim-btn\" onclick=\"trimMotor('+i+',1)\">+1</button>'+\n"
  "        '<button class=\"trim-btn\" onclick=\"trimMotor('+i+',5)\">+5</button>'+\n"
  "      '</div>'+\n"
  "      '<div class=\"btn-row\">'+\n"
  "        '<button class=\"home-btn\" onclick=\"homeMotor('+i+')\">↤ Home '+NAMES[i]+'</button>'+\n"
  "        '<button class=\"reset-btn\" onclick=\"resetTrim('+i+')\">✕ Reset Trim</button>'+\n"
  "      '</div>';\n"
  "  }\n"
  "  if(d.homeCW!==undefined){\n"
  "    document.getElementById('tog-home').checked=d.homeCW;\n"
  "    document.getElementById('home-dir-label').textContent=d.homeCW?'CW':'CCW';\n"
  "  }\n"
  "  if(d.hotMoreSteps!==undefined){\n"
  "    document.getElementById('tog-hot').checked=d.hotMoreSteps;\n"
  "    document.getElementById('hot-dir-label').textContent=d.hotMoreSteps?'ON':'OFF';\n"
  "  }\n"
  "}\n"
  "function poll(){\n"
  "  fetch('/state').then(r=>r.json()).then(d=>{\n"
  "    const v=d.tempF>-900;\n"
  "    document.getElementById('temp-big').textContent=v?d.tempF.toFixed(1)+'°F':'—';\n"
  "    document.getElementById('temp-meta').innerHTML=v\n"
  "      ?'Live sensor<br>Shown: '+(d.displayedTempF>-900?d.displayedTempF.toFixed(1)+'°F':'—')+'<br>Deadband ±0.8°F'\n"
  "      :'No sensor on pin 4';\n"
  "    buildGrid(d);\n"
  "    document.getElementById('last-update').textContent='Updated '+new Date().toLocaleTimeString();\n"
  "  }).catch(()=>{document.getElementById('last-update').textContent='Connection error';});\n"
  "}\n"
  "setInterval(poll,500);poll();\n"
  "</script>\n"
  "</body>\n"
  "</html>\n"
  "";

// ── CONVERSION HELPERS ────────────────────────────────────────
// Direction controlled by hotMoreSteps flag (set from web UI).
// hotMoreSteps=false: home=hot end, more steps=colder (DIAL_MAX-based)
// hotMoreSteps=true:  home=cold end, more steps=hotter (DIAL_MIN-based)
long tempToSteps(float degF) {
  if (degF < DIAL_MIN_F) degF = DIAL_MIN_F;
  if (degF > DIAL_MAX_F) degF = DIAL_MAX_F;
  if (hotMoreSteps)
    return (long)((degF - DIAL_MIN_F) * STEPS_PER_DEG + 0.5f);
  else
    return (long)((DIAL_MAX_F - degF) * STEPS_PER_DEG + 0.5f);
}

float stepsToTemp(long steps) {
  if (hotMoreSteps)
    return DIAL_MIN_F + (float)steps / STEPS_PER_DEG;
  else
    return DIAL_MAX_F - (float)steps / STEPS_PER_DEG;
}

// ── MOTOR CONTROL (called from loop, not ISR) ─────────────────
void motorMoveTo(int idx, long target) {
  Motor& m = motors[idx];
  if (target == m.currentSteps) return;
  bool cw = (target > m.currentSteps);
  // Set DIR pin then let ticksRemaining provide the setup settling time.
  // Do NOT use delayMicroseconds here — it blocks the ISR and stalls
  // all other motors. STEP_TICKS (~9ms) far exceeds the 10µs DIR setup.
  digitalWrite(PIN_DIR[idx], cw ? DIR_CW : DIR_CCW);
  portENTER_CRITICAL(&timerMux);
  m.targetSteps    = target;
  m.directionCW    = cw;
  m.ticksRemaining = STEP_TICKS;
  m.pulseActive    = false;
  m.state          = SYS_MOVING;
  portEXIT_CRITICAL(&timerMux);
}

void motorStartHoming(int idx, PostHomeAction next) {
  Motor& m = motors[idx];
  bool cw = homeDirCW;
  digitalWrite(PIN_DIR[idx], cw ? DIR_CW : DIR_CCW);
  portENTER_CRITICAL(&timerMux);
  m.postHomeAction  = next;
  m.directionCW     = cw;
  m.homingDone      = false;
  m.ticksRemaining  = STEP_TICKS;
  m.pulseActive     = false;
  m.state           = SYS_HOMING;
  portEXIT_CRITICAL(&timerMux);
}

// ── HTTP HANDLERS ─────────────────────────────────────────────
void handleRoot() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent_P(PAGE_HTML_0);
  server.sendContent_P(PAGE_HTML_1);
  server.sendContent_P(PAGE_HTML_2);
  server.sendContent_P(PAGE_HTML_3);
  server.sendContent("");
}

void handleHomeAll() {
  for (int i = 0; i < NUM_MOTORS; i++)
    motorStartHoming(i, POST_HOME_GO_TO_TEMP);
  lastHomeMs = millis();
  server.send(200, "text/plain", "OK");
}

void handleHomeMotor() {
  if (server.hasArg("motor")) {
    int idx = server.arg("motor").toInt();
    if (idx >= 0 && idx < NUM_MOTORS)
      motorStartHoming(idx, POST_HOME_GO_TO_TEMP);
  }
  server.send(200, "text/plain", "OK");
}

// ── Save all trim values to NVS ──────────────────────────────
void saveTrims() {
  prefs.begin("clock", false);
  for (int i = 0; i < NUM_MOTORS; i++) {
    String key = "trim" + String(i);
    prefs.putInt(key.c_str(), motors[i].trimSteps);
  }
  prefs.end();
}

void handleTrimMotor() {
  if (server.hasArg("motor") && server.hasArg("delta")) {
    int idx   = server.arg("motor").toInt();
    int delta = server.arg("delta").toInt();
    if (idx >= 0 && idx < NUM_MOTORS) {
      motors[idx].trimSteps += delta;
      if (motors[idx].trimSteps >  160) motors[idx].trimSteps =  160;
      if (motors[idx].trimSteps < -160) motors[idx].trimSteps = -160;
      if (motors[idx].state == SYS_IDLE && currentTempF > -900) {
        long tgt = tempToSteps(currentTempF) + motors[idx].trimSteps;
        if (tgt < 0)   tgt = 0;
        if (tgt > 1600) tgt = 1600;
        motorMoveTo(idx, tgt);
      }
      saveTrims();
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleResetTrim() {
  if (server.hasArg("motor")) {
    int idx = server.arg("motor").toInt();
    if (idx >= 0 && idx < NUM_MOTORS) {
      motors[idx].trimSteps = 0;
      if (motors[idx].state == SYS_IDLE && currentTempF > -900) {
        long tgt = tempToSteps(currentTempF);
        if (tgt < 0)    tgt = 0;
        if (tgt > 1600) tgt = 1600;
        motorMoveTo(idx, tgt);
      }
      saveTrims();
    }
  }
  server.send(200, "text/plain", "OK");
}

void handleResetAllTrims() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].trimSteps = 0;
    if (motors[i].state == SYS_IDLE && currentTempF > -900) {
      long tgt = tempToSteps(currentTempF);
      if (tgt < 0)    tgt = 0;
      if (tgt > 1600) tgt = 1600;
      motorMoveTo(i, tgt);
    }
  }
  saveTrims();
  server.send(200, "text/plain", "OK");
}

void handleSetConfig() {
  bool changed = false;
  if (server.hasArg("homeCW")) {
    homeDirCW    = server.arg("homeCW").toInt() != 0;
    changed = true;
  }
  if (server.hasArg("hotMoreSteps")) {
    hotMoreSteps = server.arg("hotMoreSteps").toInt() != 0;
    changed = true;
  }
  if (changed) {
    prefs.begin("clock", false);        // open NVS namespace "clock"
    prefs.putBool("homeCW", homeDirCW);
    prefs.putBool("hotSteps", hotMoreSteps);
    prefs.end();
    Serial.printf("Config saved — homeCW=%d hotMoreSteps=%d\n",
                  homeDirCW, hotMoreSteps);
  }
  server.send(200, "text/plain", "OK");
}

void handleState() {
  const char* stateNames[] = { "Homing", "Moving", "Idle" };
  String json = "{";
  json += "\"tempF\":"          + String(currentTempF, 1)    + ",";
  json += "\"displayedTempF\":" + String(displayedTempF, 1)  + ",";
  json += "\"motors\":[";
  for (int i = 0; i < NUM_MOTORS; i++) {
    Motor& m = motors[i];
    if (i > 0) json += ",";
    json += "{";
    json += "\"state\":\""    + String(stateNames[m.state]) + "\",";
    json += "\"steps\":"      + String(m.currentSteps)      + ",";
    json += "\"posDegF\":"    + String(stepsToTemp(m.currentSteps), 1) + ",";
    json += "\"targetDegF\":" + String(stepsToTemp(m.targetSteps), 1) + ",";
    json += "\"trim\":"       + String(m.trimSteps);
    json += "}";
  }
  json += "],\"homeCW\":" + String(homeDirCW ? "true" : "false") + ",";
  json += "\"hotMoreSteps\":" + String(hotMoreSteps ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

// ── SETUP ─────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(PIN_PUL[i],   OUTPUT);
    pinMode(PIN_DIR[i],   OUTPUT);
    pinMode(PIN_LIMIT[i], INPUT_PULLUP);
    digitalWrite(PIN_PUL[i], PUL_IDLE);
    digitalWrite(PIN_DIR[i], DIR_CCW);
  }

  sensors.begin();

  // Boot temperature (single sample, range-checked only)
  sensors.requestTemperatures();
  float t = sensors.getTempFByIndex(0);
  if (tempIsValid(t)) {
    currentTempF = t;
    lastRawTempF = t;
  }
  lastTempMs = millis();

  // ── Load saved direction config from NVS ──────────────────
  prefs.begin("clock", true);           // open read-only
  homeDirCW    = prefs.getBool("homeCW",    true);   // default CW
  hotMoreSteps = prefs.getBool("hotSteps",  false);  // default false
  for (int i = 0; i < NUM_MOTORS; i++) {
    String key = "trim" + String(i);
    motors[i].trimSteps = prefs.getInt(key.c_str(), 0);
  }
  prefs.end();
  Serial.printf("Config loaded — homeCW=%d hotMoreSteps=%d trims=%d,%d,%d,%d\n",
                homeDirCW, hotMoreSteps,
                motors[0].trimSteps, motors[1].trimSteps,
                motors[2].trimSteps, motors[3].trimSteps);

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  server.on("/",           HTTP_GET,  handleRoot);
  server.on("/state",      HTTP_GET,  handleState);
  server.on("/home_all",   HTTP_POST, handleHomeAll);
  server.on("/home_motor", HTTP_POST, handleHomeMotor);
  server.on("/trim_motor",      HTTP_POST, handleTrimMotor);
  server.on("/reset_trim",      HTTP_POST, handleResetTrim);
  server.on("/reset_all_trims", HTTP_POST, handleResetAllTrims);
  server.on("/set_config",      HTTP_POST, handleSetConfig);
  server.begin();

  // ── Start hardware timer ─────────────────────────────────────
  // Timer 0, prescaler 80 → 1 tick = 1µs at 80MHz.

  // Hardware timer: fires every ISR_TICK_US (10µs).
  // 1MHz base frequency → 1 tick = 1µs, alarm at ISR_TICK_US ticks.
  stepTimer = timerBegin(1000000);
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, ISR_TICK_US, true, 0);

  Serial.println("4-motor clock started — homing all motors…");

  // Home all motors on boot
  for (int i = 0; i < NUM_MOTORS; i++)
    motorStartHoming(i, POST_HOME_GO_TO_TEMP);
  lastHomeMs = millis();
}

// ── LOOP ──────────────────────────────────────────────────────
// loop() handles WiFi, temperature, and state decisions only.
// It never touches the stepper pins — the ISR owns those.
void loop() {
  server.handleClient();

  uint32_t now = millis();

  // ── Check for completed homings and act on them ──────────────
  // Snapshot all homingDone flags first, clear them, then act.
  // This keeps motorMoveTo() calls out of any nested lock context
  // so the ISR is never blocked while we set up the next move.
  bool didHome[NUM_MOTORS] = {};
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motors[i].homingDone) {
      motors[i].homingDone = false;
      didHome[i] = true;
      if (i == 0) lastHomeMs = millis();
    }
  }
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (didHome[i] && motors[i].postHomeAction == POST_HOME_GO_TO_TEMP
        && currentTempF > -900) {
      long tgt = tempToSteps(currentTempF) + motors[i].trimSteps;
      if (tgt < 0)    tgt = 0;
      if (tgt > 1600) tgt = 1600;
      motorMoveTo(i, tgt);
      displayedTempF = currentTempF;
    }
  }

  // ── Temperature read every 2 s ───────────────────────────────
  if ((uint32_t)(now - lastTempMs) >= TEMP_INTERVAL_MS) {
    lastTempMs = now;
    sensors.requestTemperatures();
    float t = sensors.getTempFByIndex(0);

    if (tempIsValid(t)) {
      // Two consecutive readings within 3°F required (glitch filter)
      if (lastRawTempF > -900 && fabsf(t - lastRawTempF) <= 3.0f) {
        currentTempF = t;
      }
      lastRawTempF = t;
    } else {
      Serial.printf("Temp rejected: %.1f°F (out of range)\n", t);
      lastRawTempF = -999.0f;
    }

    // Move all idle motors if temp changed beyond deadband
    if (currentTempF > -900) {
      if (displayedTempF < -900 ||
          fabsf(currentTempF - displayedTempF) >= DEADBAND_F) {
        bool anyMoved = false;
        for (int i = 0; i < NUM_MOTORS; i++) {
          if (motors[i].state == SYS_IDLE) {
            long tgt = tempToSteps(currentTempF) + motors[i].trimSteps;
            if (tgt < 0)   tgt = 0;
            if (tgt > 1600) tgt = 1600;
            if (tgt != motors[i].currentSteps) {
              motorMoveTo(i, tgt);
              anyMoved = true;
            }
          }
        }
        if (anyMoved) {
          displayedTempF = currentTempF;
          Serial.printf("Moving all to %.1f°F\n", currentTempF);
        }
      }
    }
  }

  // ── Daily re-home ────────────────────────────────────────────
  bool allIdle = true;
  for (int i = 0; i < NUM_MOTORS; i++)
    if (motors[i].state != SYS_IDLE) { allIdle = false; break; }

  if (allIdle && (uint32_t)(now - lastHomeMs) >= DAILY_HOME_INTERVAL_MS) {
    Serial.println("Daily re-home triggered");
    for (int i = 0; i < NUM_MOTORS; i++)
      motorStartHoming(i, POST_HOME_GO_TO_TEMP);
    lastHomeMs = millis();
  }
}
