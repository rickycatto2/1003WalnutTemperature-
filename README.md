# Infinity Signs — Vintage Temperature Clock

A retrofit of a 4-face vintage temperature gauge using stepper motors controlled by an ESP32. Each of the four dials (North, East, South, West) is driven by an independent stepper motor, all reading the same DS18B20 temperature sensor. The system hosts a mobile-friendly web UI over its own WiFi access point for monitoring and calibration.

---

## Hardware

### Controller
- **ESP32 WROOM-32**
- Arduino IDE with ESP32 board package (core 3.x)

### Motors — one per dial
- **STEPPERONLINE Nema 17 Bipolar Stepper** — 2A, 59Ncm, 48mm body
- 200 steps/rev base, run at **1/8 microstep = 1600 steps/rev**

### Drivers — one per motor
- **TB6600 Stepper Motor Driver**
- PUL+ and DIR+ tied to **3.3V** (active-LOW control from ESP32)
- ENA left unconnected (always enabled)

### Temperature sensor
- **DS18B20** — one-wire digital temperature sensor

### Limit switches
- One per motor, wired normally-open
- Triggered at the **+135°F end** of each dial
- Use hose clamps or similar mechanical triggers on the dial shaft

---

## Wiring

### ESP32 Pin Assignments

| Signal | North (M1) | East (M2) | South (M3) | West (M4) |
|--------|-----------|-----------|------------|-----------|
| PUL−   | GPIO 25   | GPIO 32   | GPIO 22    | GPIO 16   |
| DIR−   | GPIO 26   | GPIO 33   | GPIO 23    | GPIO 17   |
| LIMIT  | GPIO 27   | GPIO 14   | GPIO 21    | GPIO 18   |

| Signal | Pin |
|--------|-----|
| DS18B20 data | GPIO 4 |

**Side A pins used:** 3V3, 25, 26, 27, 32, 33, 14, GND

**Side B pins used:** 21, 18, 17, 16, 4, GND

> **Pins deliberately avoided:**
> - GPIO 12/13 — GPIO 12 is a boot-strap pin. If HIGH at reset the ESP32 tries to boot from 1.8V flash and hangs.
> - GPIO 34/35 — input-only, no internal pull-up resistor, float at boot.
> - GPIO 0/2/5/15 — boot-strapping or PWM-at-boot issues.

### TB6600 Wiring (per driver)

| TB6600 terminal | Connect to |
|-----------------|-----------|
| PUL+            | 3.3V |
| PUL−            | ESP32 PUL pin |
| DIR+            | 3.3V |
| DIR−            | ESP32 DIR pin |
| ENA+/ENA−       | Leave unconnected |
| A+, A−          | Motor coil A |
| B+, B−          | Motor coil B |
| VCC             | 12–24V DC power supply |
| GND             | Power supply ground |

> Active-LOW logic: the ESP32 pulls PUL− and DIR− LOW to assert. A step fires on the **falling edge** of PUL−.

### Limit Switch Wiring (per switch)

| Switch terminal | Connect to |
|-----------------|-----------|
| Common (C)      | GND |
| Normally Open   | ESP32 LIMIT pin |

The ESP32 enables `INPUT_PULLUP` on each LIMIT pin so it reads HIGH when open and LOW when the switch is triggered.

---

## TB6600 DIP Switch Settings

Set all four drivers identically:

| Switch | Position | Function |
|--------|----------|---------|
| SW1    | ON       | Microstep bit 1 |
| SW2    | OFF      | Microstep bit 2 |
| SW3    | ON       | Microstep bit 3 → **1/8 microstep = 1600 steps/rev** |
| SW4    | OFF      | Current bit 1 |
| SW5    | OFF      | Current bit 2 |
| SW6    | ON       | Current bit 3 → **2.5A peak** |

> "Switch down = ON" on the TB6600 label.
> 2.5A is appropriate for the 2A-rated Nema 17. Running at 1A (previous setting) caused stuttering due to insufficient holding torque.

---

## Software Setup

### Arduino Libraries Required

Install all via Arduino IDE Library Manager:

| Library | Author |
|---------|--------|
| OneWire | Paul Stoffregen |
| DallasTemperature | Miles Burton |

Built-in ESP32 libraries used (no install needed): `WiFi`, `WebServer`, `Preferences`.

### Secrets File

The WiFi password is kept out of the repository. Before compiling:

1. Copy `secrets.h.example` to `secrets.h`
2. Fill in your AP password in `secrets.h`
3. `secrets.h` is listed in `.gitignore` and will never be committed

```cpp
// secrets.h
#define AP_SSID "Infinity Signs - (816) 252 3337"
#define AP_PASS  "your_password_here"
```

### Compiling

1. Open `stepper_clock_4motor.ino` in Arduino IDE
2. Select board: **ESP32 Dev Module**
3. Select the correct COM port
4. Click Upload

---

## Dial Specifications

| Parameter | Value |
|-----------|-------|
| Temperature range | −25°F to +135°F |
| Span | 160°F |
| Steps per revolution | 1600 (1/8 microstep) |
| Steps per degree | 10.0 |
| Deadband | 0.8°F (8 steps minimum move) |
| Motor speed | ~15 seconds per full revolution |
| Dial direction | Numbers painted CCW — limit switch at +135°F end |

---

## Stepper Architecture

Stepping is driven by a **hardware timer ISR** (ESP32 Timer 0, 1MHz tick rate). The ISR fires every **10µs** independently of the main loop.

Each step is a two-tick sequence:
- **Tick N:** countdown reaches zero → assert PUL− LOW
- **Tick N+1:** release PUL− HIGH → 10µs pulse committed, step counted, counter reloaded

This means WiFi, web server responses, and temperature reads **cannot cause stepping stutter** — the ISR is always running at exactly the right rate regardless of what `loop()` is doing.

To adjust speed, change `STEP_TICKS` in the sketch. Every ~67 ticks ≈ 1 sec/rev change:

```cpp
#define STEP_TICKS  935UL   // ~15 sec/rev — increase = slower, decrease = faster
```

---

## Web UI

Connect to the WiFi AP (`Infinity Signs - (816) 252 3337`) then navigate to **http://192.168.4.1**

The UI is mobile-first and designed for phone use on-site.

### Temperature display
Live DS18B20 reading with a two-sample confirmation filter (rejects single-sample spikes). Requires two consecutive readings within 3°F of each other before accepting.

### Per-motor cards (North / East / South / West)
Each card shows current position, target, step count, and trim offset. Controls:
- **Trim ±1 / ±5** — nudge the dial position to correct mechanical offset from the limit switch. Range ±160 steps (±16°F). Saved to flash immediately.
- **↤ Home** — re-home that individual motor
- **✕ Reset Trim** — zero out trim for that motor (with confirmation)

### Direction Settings
Two toggles that control motor direction and are saved to flash:

| Toggle | OFF | ON |
|--------|-----|----|
| **Home direction** | CCW | CW |
| **Hotter = more steps** | hotter → fewer steps (toward home) | hotter → more steps (away from home) |

These are the key settings to get right for your specific installation — if the motor homes the wrong way, flip the Home Direction toggle. If temperature moves the dial the wrong way, flip Hotter = more steps.

### Global Controls
- **↤ Re-home All Motors** — homes all 4 simultaneously
- **↻ Refresh** — manual poll
- **✕ Reset All Trims to Zero** — zeroes all 4 trims (with confirmation)

---

## NVS Persistent Storage

The following settings survive power cycles (stored in ESP32 flash via the `Preferences` library under namespace `"clock"`):

| NVS Key | Type | Description |
|---------|------|-------------|
| `homeCW` | bool | Home direction (true = CW) |
| `hotSteps` | bool | Hotter = more steps |
| `trim0` | int | North trim offset |
| `trim1` | int | East trim offset |
| `trim2` | int | South trim offset |
| `trim3` | int | West trim offset |

On first boot (no saved values) all trims default to 0, home direction defaults to CW, and hotter-more-steps defaults to false.

The Serial monitor (115200 baud) prints all loaded values on boot:
```
Config loaded — homeCW=1 hotMoreSteps=0 trims=0,0,0,0
```

---

## Daily Re-home

Every 24 hours (while all motors are idle) the system automatically re-homes all four motors and returns them to the current temperature position. This corrects any accumulated step drift over time.

---

## Calibration Notes

### Limit switch placement
The limit switches are triggered by hose clamps on each dial shaft at the +135°F position. Because hose clamp placement is not perfectly repeatable, the per-motor **trim** adjustment compensates for any angular offset. Adjust trim after each re-home until all four dials read the same temperature accurately.

### Trim procedure (after installation)
1. Power on — all motors home to their limit switches
2. Compare each dial reading against a reference thermometer
3. Use the web UI trim buttons to nudge each dial to the correct position
4. Trim is saved automatically — no further action needed

---

## Files

| File | Description |
|------|-------------|
| `stepper_clock_4motor.ino` | Main production sketch |
| `motor_test.ino` | Test sketch — 1 full CW rotation then 1 CCW on repeat, no WiFi or temperature logic |
| `secrets.h.example` | Template for WiFi credentials |
| `secrets.h` | Your actual credentials — **not committed, listed in .gitignore** |
| `README.md` | This file |

---

## Troubleshooting

**ESP32 won't boot with motors connected**
GPIO 12 is a boot-strap pin and must be LOW at reset. If any TB6600 pulls it HIGH through the optocoupler, the chip tries to boot from 1.8V flash and hangs. The sketch avoids GPIO 12/13 for this reason — double-check your wiring matches the pin table above.

**Motor homes immediately without moving**
The limit switch pin is reading LOW before the switch is triggered. Check for a floating or mis-wired LIMIT pin. GPIO 34/35 are particularly prone to this as they have no internal pull-up — the sketch avoids them for this reason.

**Stepping is stuttery or uneven**
Ensure the TB6600 current setting is at least 2.0A (SW6=ON). Running at 1A causes the rotor to oscillate at each step position rather than holding firmly.

**Dial moves wrong direction after homing**
Use the **Home Direction** toggle in the web UI. The correct direction depends on how the motor coils are wired to the TB6600 terminals — swapping A+/A− or B+/B− will also reverse physical direction.

**Temperature reading shows −196°F or similar**
DS18B20 CRC error or loose connection. The two-sample confirmation filter will reject it but check the data line connection on GPIO 4 and ensure a 4.7kΩ pull-up resistor is present between the data line and 3.3V.
