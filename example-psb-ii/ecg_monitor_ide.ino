/*
 * ECG Heart Rate Zone & Alarm Monitor
 * Sensor : AD8232 single-lead ECG front-end
 * Board  : Arduino Uno / Nano (any 5V ATmega328) - also works on 3.3V boards
 *
 * Behaviour
 *   Buzzer     : short beep on every detected heartbeat (hospital-monitor click)A
 *   Blue LED   : blinks while the electrodes are disconnected (Leads-Off)
 *   Green LED  : blinks in rhythm with a NORMAL heart rate
 *   Red LED    : solid ON + continuous buzzer if HR is dangerous
 *                (Tachycardia > 120 bpm  or  Bradycardia < 50 bpm)
 *
 * Serial : streams one CSV line per sample at 115200 baud, ~200 Hz:
 *              raw,leadsOff,beat,bpm
 *          raw       = 0..1023 ADC reading from AD8232 OUTPUT
 *          leadsOff  = 1 if an electrode is off, else 0
 *          beat      = 1 on the exact sample an R-peak was detected, else 0
 *          bpm       = last computed beats-per-minute (0 until first R-R)
 *
 *  ----------------------------------------------------------------
 *  WIRING (Arduino Uno / Nano)
 *  ----------------------------------------------------------------
 *  AD8232 module        Arduino
 *    3.3V / GND     ->  3.3V / GND      (power the module from 3.3V)
 *    OUTPUT         ->  A0              (analog ECG signal)
 *    LO+            ->  D10             (leads-off detect +)
 *    LO-            ->  D11             (leads-off detect -)
 *    SDN            ->  (leave open, or tie to 3.3V to keep awake)
 *
 *  Buzzer (ACTIVE buzzer, has its own oscillator)
 *    + (signal)     ->  D8
 *    - (GND)        ->  GND
 *
 *  LEDs (each through a 220-330 ohm resistor to GND)
 *    Green  LED     ->  D3
 *    Blue   LED     ->  D4
 *    Red    LED     ->  D5
 *  ----------------------------------------------------------------
 */

// ---------- Pin map ----------
const uint8_t PIN_ECG    = A0;   // AD8232 OUTPUT
const uint8_t PIN_LO_P   = 10;   // AD8232 LO+
const uint8_t PIN_LO_M   = 11;   // AD8232 LO-
const uint8_t PIN_BUZZER = 8;    // active buzzer +
const uint8_t PIN_GREEN  = 3;    // normal-rhythm LED
const uint8_t PIN_BLUE   = 4;    // leads-off LED
const uint8_t PIN_RED    = 5;    // danger LED

// ---------- Timing ----------
const uint16_t SAMPLE_HZ   = 200;              // sampling rate
const uint16_t SAMPLE_US   = 1000000 / SAMPLE_HZ;
const uint16_t BEEP_MS     = 40;               // beep length per beat
const uint16_t GREEN_MS    = 40;               // green flash length per beat
const uint16_t BLUE_BLINK  = 250;              // leads-off blink half-period

// ---------- Heart-rate zones (bpm) ----------
const int BPM_BRADY = 50;    // below this = bradycardia (danger)
const int BPM_TACHY = 120;   // above this = tachycardia (danger)

// ---------- Beat detection (dynamic threshold) ----------
// AD8232 idles around mid-scale (~512). R-peaks punch upward.
float   baseline   = 512.0;    // slow-moving DC baseline
float   envelope   = 40.0;     // running estimate of peak height above baseline
const float THRESH_FRAC = 0.6; // fire at 60% of the running envelope
const uint16_t REFRACTORY_MS = 250; // min gap between beats (=> max 240 bpm)

unsigned long lastSampleUs = 0;
unsigned long lastBeatMs   = 0;
unsigned long beepUntilMs  = 0;
unsigned long greenUntilMs = 0;
unsigned long blueToggleMs = 0;
bool          blueState    = false;
bool          aboveThresh  = false;

int  bpm = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LO_P, INPUT);
  pinMode(PIN_LO_M, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_GREEN,  OUTPUT);
  pinMode(PIN_BLUE,   OUTPUT);
  pinMode(PIN_RED,    OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
}

void loop() {
  unsigned long nowUs = micros();
  if (nowUs - lastSampleUs < SAMPLE_US) return;   // hold the sample rate
  lastSampleUs = nowUs;
  unsigned long nowMs = millis();

  // ---- 1. Leads-off detection ----
  bool leadsOff = (digitalRead(PIN_LO_P) == HIGH) || (digitalRead(PIN_LO_M) == HIGH);

  int raw = leadsOff ? 512 : analogRead(PIN_ECG);   // flatline when disconnected

  // ---- 2. Beat detection (only when connected) ----
  bool beat = false;
  if (!leadsOff) {
    // slow baseline tracking (removes DC drift for the trigger only)
    baseline += (raw - baseline) * 0.002;
    float amp = raw - baseline;
    if (amp < 0) amp = 0;

    float threshold = envelope * THRESH_FRAC;

    if (!aboveThresh && amp > threshold && (nowMs - lastBeatMs) > REFRACTORY_MS) {
      // rising edge through the dynamic ceiling => R-peak
      aboveThresh = true;
      beat = true;

      unsigned long rr = nowMs - lastBeatMs;   // R-R interval in ms
      lastBeatMs = nowMs;
      if (rr > 250 && rr < 3000) {             // 20..240 bpm sanity window
        bpm = (int)(60000.0 / rr);
      }

      // adapt the envelope toward the measured peak height
      envelope += (amp - envelope) * 0.25;

      // schedule feedback
      beepUntilMs  = nowMs + BEEP_MS;
      greenUntilMs = nowMs + GREEN_MS;
    }
    if (amp < threshold * 0.5) aboveThresh = false;   // reset for next beat
  } else {
    bpm = 0;                 // no meaningful rate while disconnected
    aboveThresh = false;
  }

  // ---- 3. Danger zone check ----
  bool danger = (!leadsOff) && bpm > 0 && (bpm > BPM_TACHY || bpm < BPM_BRADY);

  // ---- 4. Outputs ----
  // Blue: blink while leads off
  if (leadsOff) {
    if (nowMs - blueToggleMs > BLUE_BLINK) { blueToggleMs = nowMs; blueState = !blueState; }
    digitalWrite(PIN_BLUE, blueState);
  } else {
    digitalWrite(PIN_BLUE, LOW);
    blueState = false;
  }

  // Red: solid on danger
  digitalWrite(PIN_RED, danger ? HIGH : LOW);

  // Green: flash per beat only when rate is normal
  bool greenOn = (!danger) && (nowMs < greenUntilMs);
  digitalWrite(PIN_GREEN, greenOn ? HIGH : LOW);

  // Buzzer: continuous on danger, otherwise a short click per beat
  if (danger) {
    digitalWrite(PIN_BUZZER, HIGH);
  } else {
    digitalWrite(PIN_BUZZER, (nowMs < beepUntilMs) ? HIGH : LOW);
  }

  // ---- 5. Stream to Python ----
  Serial.print(raw);
  Serial.print(',');
  Serial.print(leadsOff ? 1 : 0);
  Serial.print(',');
  Serial.print(beat ? 1 : 0);
  Serial.print(',');
  Serial.println(bpm);
}
