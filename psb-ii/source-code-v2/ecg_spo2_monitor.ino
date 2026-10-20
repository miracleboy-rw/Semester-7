/*
 * =====================================================================
 *  MONITOR PASIEN GABUNGAN : EKG (AD8232) + SpO2/Pulse (MAX30100)
 *  Board   : ESP32 (ESP32 DevKit V1 / ESP32-WROOM-32)
 *  Display : OLED SSD1306 128x64 I2C
 * =====================================================================
 *
 *  Gabungan dari 2 proyek sebelumnya:
 *    - ecg_monitor_ide.ino        (AD8232, deteksi R-peak, buzzer, LED)
 *    - pulse_oximeter_monitor.ino (I2C scanner -> kini jadi diagnostik boot)
 *      + driver MAX30100 langsung via register (TANPA library MAX30100)
 *
 *  Library yang dibutuhkan (Library Manager Arduino IDE):
 *    - Adafruit SSD1306
 *    - Adafruit GFX Library   (otomatis ikut terpasang)
 *    - Adafruit BusIO         (otomatis ikut terpasang)
 *    - PubSubClient           (Nick O'Leary; hanya bila ENABLE_MQTT = 1)
 *  Board package: "esp32 by Espressif Systems"  -> pilih "ESP32 Dev Module"
 *
 *  ---------------------------------------------------------------
 *  PEMASANGAN KABEL (detail lengkap ada di README.md)
 *  ---------------------------------------------------------------
 *   Modul / Komponen          Pin modul    ->  Pin ESP32
 *   AD8232 (EKG)              3.3V         ->  3V3
 *                             GND          ->  GND
 *                             OUTPUT       ->  GPIO34 (ADC1_CH6, input-only)
 *                             LO+          ->  GPIO25
 *                             LO-          ->  GPIO26
 *                             SDN          ->  dibiarkan kosong
 *   MAX30100 (SpO2)           VIN          ->  3V3
 *                             GND          ->  GND
 *                             SDA          ->  GPIO21   (I2C bersama OLED)
 *                             SCL          ->  GPIO22   (I2C bersama OLED)
 *                             INT          ->  tidak dipakai
 *   OLED SSD1306 I2C          VCC          ->  3V3
 *                             GND          ->  GND
 *                             SDA          ->  GPIO21   (I2C bersama MAX30100)
 *                             SCL          ->  GPIO22   (I2C bersama MAX30100)
 *   Buzzer aktif              (+)          ->  GPIO27   (lihat README: sebaiknya lewat transistor)
 *                             (-)          ->  GND
 *   LED Hijau  -> resistor 220-330 ohm ->  GPIO18
 *   LED Biru   -> resistor 220-330 ohm ->  GPIO19
 *   LED Merah  -> resistor 220-330 ohm ->  GPIO23
 *
 *  ---------------------------------------------------------------
 *  PERILAKU
 *  ---------------------------------------------------------------
 *   Buzzer     : bunyi pendek tiap detak jantung; bunyi terus saat BAHAYA
 *   LED Hijau  : berkedip tiap detak (kondisi normal)
 *   LED Biru   : berkedip jika elektrode EKG lepas (Leads-Off) ATAU jari
 *                tidak ada di sensor MAX30100
 *   LED Merah  : menyala terus saat BAHAYA:
 *                  HR/PR > 120 bpm (takikardia), HR/PR < 50 bpm (bradikardia)
 *                  atau SpO2 < 90 %
 *   OLED       : gelombang EKG + HR + SpO2 + PR + status
 *
 *  ---------------------------------------------------------------
 *  PROTOKOL SERIAL (230400 baud, 1 baris ASCII per sampel)
 *  ---------------------------------------------------------------
 *   E,<raw>,<leadsOff>,<beat>,<bpm>\n      200 Hz  (EKG)
 *        raw      = 0..4095 (ADC 12-bit AD8232), datar di 2048 saat leads-off
 *        leadsOff = 1 jika elektrode lepas
 *        beat     = 1 tepat pada sampel R-peak terdeteksi
 *        bpm      = HR dari EKG (0 = belum valid)
 *   P,<ir>,<red>,<bpm>,<spo2>\n            100 Hz  (PPG)
 *        ir/red   = nilai mentah MAX30100 (16-bit)
 *        bpm      = pulse rate dari PPG (0 = belum valid)
 *        spo2     = estimasi SpO2 % (0 = belum valid)
 *   # ...                                 komentar/diagnostik (diabaikan parser)
 *
 *  ---------------------------------------------------------------
 *  MQTT (WiFi): alat dapat berada JAUH dari monitor
 *  ---------------------------------------------------------------
 *   Isi WIFI_SSID / WIFI_PASS / MQTT_HOST pada bagian KONFIGURASI JARINGAN.
 *   Alat menerbitkan ke broker; aplikasi monitor berlangganan topik yang sama.
 *
 *   Topik:
 *     monitor/<id>/stream   biner, tiap 100 ms: 20 sampel EKG + ~10 sampel PPG
 *     monitor/<id>/vitals   JSON 1 Hz: hr, pr, spo2, loff, foff, danger, mask, rssi, ...
 *     monitor/<id>/alarm    JSON (retained) tiap status alarm berubah
 *     monitor/<id>/status   "online" / "offline" (retained; "offline" = Last Will)
 *
 *   Format paket stream (little-endian, header 20 byte):
 *     0  u8  versi (=1)         1  u8  flags: b0=leadsOff b1=fingerOff b2=danger
 *     2  u32 nomor urut pesan   6  u32 indeks sampel EKG pertama
 *     10 u8  nE                 11 u32 indeks sampel PPG pertama
 *     15 u8  nP                 16 u8  pulse rate   17 u8 SpO2   18 u16 HR dari EKG
 *     20 ...  nE x u16 (EKG 12-bit), lalu nP x (u16 IR, u16 RED)
 *   Penerima menyusun gelombang dari INDEKS sampel (bukan dari waktu tiba),
 *   sehingga jitter jaringan tidak merusak bentuk gelombang maupun BPM.
 *
 *  Arsitektur:  core 1 (loop)  : sampling EKG tepat 200 Hz, deteksi detak,
 *                                LED/buzzer, semua output Serial
 *               core 0 (task)  : SEMUA akses I2C (MAX30100 + OLED), sehingga
 *                                refresh OLED yang lambat tidak mengganggu
 *                                waktu sampling EKG. Task jaringan (WiFi/MQTT)
 *                                juga di core 0 dan hanya memakai antrean:
 *                                koneksi lambat/putus TIDAK menunda sampling EKG.
 *
 *  CATATAN: ini proyek hobi/edukasi, BUKAN alat medis.
 * =====================================================================
 */

// =====================================================================
//  KONFIGURASI JARINGAN (WiFi + MQTT)  -- EDIT BAGIAN INI
// =====================================================================
#define ENABLE_MQTT        1      // 1 = kirim data ke broker MQTT lewat WiFi; 0 = hanya USB serial
#define SERIAL_STREAM      1      // 1 = tetap kirim baris E,/P, ke USB serial (set 0 bila dipasang jauh)

#define WIFI_SSID          "ISI_NAMA_WIFI"        // selama masih nilai ini, MQTT tidak diaktifkan
#define WIFI_PASS          "ISI_PASSWORD_WIFI"

#define MQTT_HOST          "192.168.1.10"         // alamat broker (IP atau nama domain)
#define MQTT_PORT          1883                   // 1883 = tanpa TLS, 8883 = TLS
#define MQTT_USER          ""                     // kosongkan bila broker tanpa login
#define MQTT_PASS          ""
#define MQTT_USE_TLS       0                      // 1 = enkripsi TLS (wajib bila lewat internet)
#define MQTT_TLS_INSECURE  0                      // 1 = TLS tanpa verifikasi sertifikat (HANYA uji coba)
#define MQTT_PREFIX        "monitor"
#define DEVICE_ID          "esp32-01"             // unik per alat -> topik monitor/<id>/...
#define MQTT_MAX_AGE_MS    2000                   // paket lebih tua dari ini dibuang (utamakan data live)
#define NET_QUEUE_DEPTH    80                     // antrean paket: 80 x 100 ms = 8 detik

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#if ENABLE_MQTT
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#endif

#if ENABLE_MQTT && MQTT_USE_TLS && !MQTT_TLS_INSECURE
// Sertifikat CA (root) yang menandatangani sertifikat broker, format PEM.
// Broker dengan Let's Encrypt: tempel "ISRG Root X1" dari letsencrypt.org/certificates
static const char MQTT_CA_CERT[] = R"EOF(
-----BEGIN CERTIFICATE-----
ISI_SERTIFIKAT_CA_ROOT_BROKER_DI_SINI
-----END CERTIFICATE-----
)EOF";
#endif

// ======================= PIN =======================
const uint8_t PIN_SDA    = 21;   // I2C (OLED + MAX30100)
const uint8_t PIN_SCL    = 22;
const uint8_t PIN_ECG    = 34;   // AD8232 OUTPUT (ADC1, input-only)
const uint8_t PIN_LO_P   = 25;   // AD8232 LO+
const uint8_t PIN_LO_M   = 26;   // AD8232 LO-
const uint8_t PIN_BUZZER = 27;   // buzzer aktif
const uint8_t PIN_GREEN  = 18;   // LED hijau
const uint8_t PIN_BLUE   = 19;   // LED biru
const uint8_t PIN_RED    = 23;   // LED merah

// ======================= KONFIGURASI =======================
#define SERIAL_BAUD        230400
#define OLED_ADDR          0x3C
#define OLED_W             128
#define OLED_H             64
#define OLED_PERIOD_MS     80          // ~12 fps

const uint16_t ECG_HZ        = 200;
const uint32_t ECG_PERIOD_US = 1000000UL / ECG_HZ;
const int      ADC_MID       = 2048;    // titik tengah ADC 12-bit

const uint16_t BEEP_MS    = 40;         // lama bunyi per detak
const uint16_t GREEN_MS   = 40;         // lama kedip hijau per detak
const uint16_t BLUE_BLINK = 250;        // setengah periode kedip biru

// Batas alarm (sama dengan di program Python)
const int BPM_LOW   = 50;               // < ini = bradikardia
const int BPM_HIGH  = 120;              // > ini = takikardia
const int SPO2_LOW  = 90;               // < ini = hipoksemia

// ---- deteksi R-peak EKG ----
const float    ECG_THRESH_FRAC = 0.6f;  // pemicu = 60% envelope
const float    ECG_ENV_INIT    = 160.0f;// (hitungan ADC 12-bit)
const float    ECG_ENV_MIN     = 80.0f;
const uint16_t ECG_REFRACTORY_MS = 250; // jeda minimum antar detak (maks 240 bpm)

// ---- MAX30100 ----
#define MAX_ADDR            0x57
#define IR_FINGER_THRESH    5000        // IR mentah < ini = tidak ada jari
#define PPG_HZ              100
#define PPG_DT_MS           (1000 / PPG_HZ)
#define PPG_SETTLE_SAMPLES  300         // abaikan 3 dtk pertama setelah jari terpasang
#define PPG_MIN_ENV         15.0f       // amplitudo minimum agar dianggap detak

// Register MAX30100
#define REG_INT_STATUS  0x00
#define REG_FIFO_WR     0x02
#define REG_FIFO_OVF    0x03
#define REG_FIFO_RD     0x04
#define REG_FIFO_DATA   0x05
#define REG_MODE        0x06
#define REG_SPO2_CFG    0x07
#define REG_LED_CFG     0x09
#define REG_PART_ID     0xFF

// ======================= TIPE & STATE BERSAMA =======================
struct PpgSample {
  uint16_t ir;
  uint16_t red;
  uint8_t  bpm;
  uint8_t  spo2;
  uint32_t idx;                                          // nomor urut sampel PPG sejak boot
};

Adafruit_SSD1306 display(OLED_W, OLED_H, &Wire, -1, 400000UL, 400000UL);
QueueHandle_t    ppgQ = NULL;

// Dipakai lintas core (variabel 32-bit/8-bit sejajar = atomik di ESP32)
volatile bool     gOledOk      = false;
volatile bool     gMaxOk       = false;
volatile bool     gLeadsOff    = true;
volatile bool     gFingerOff   = true;
volatile bool     gDanger      = false;
volatile int      gEcgBpm      = 0;
volatile uint8_t  gPpgBpm      = 0;
volatile uint8_t  gSpo2        = 0;
volatile uint32_t gBeatUntilMs = 0;
volatile uint32_t gPpgBeatCount = 0;
volatile uint8_t  gDangerMask  = 0;                      // b0 = HR EKG, b1 = PR, b2 = SpO2 rendah
volatile bool     gNetEnabled  = false;
volatile uint8_t  gNetState    = 0;                      // 0 = WiFi putus, 1 = WiFi OK, 2 = MQTT OK
volatile uint32_t gNetSent     = 0;
volatile uint32_t gNetDropped  = 0;
QueueHandle_t     netQ         = NULL;

// Gelombang EKG untuk OLED (kolom y; 255 = kosong)
#define WAVE_W      80
#define WAVE_TOP    12
#define WAVE_BOT    63
#define WAVE_MID    38
#define WAVE_HALF   24
#define WAVE_DECIM  8                 // 200 Hz / 8 = 25 titik/detik -> 80 px = 3,2 detik
volatile uint8_t  waveY[WAVE_W];
volatile uint8_t  waveIdx = 0;

// ======================= STATE EKG (core 1) =======================
static float    ecgBaseline = ADC_MID;
static float    ecgEnvelope = ECG_ENV_INIT;
static float    ecgPeakAmp  = 0;
static bool     ecgAbove    = false;
static bool     ecgHaveBeat = false;
static uint32_t ecgLastBeatMs = 0;
static int      ecgBpm      = 0;

static uint32_t beepUntilMs  = 0;
static uint32_t greenUntilMs = 0;
static uint32_t blueToggleMs = 0;
static bool     blueState    = false;

static float    decBest = 0;
static uint8_t  decCount = 0;

// ======================= STATE PPG (core 0) =======================
static uint8_t  ppgOffCnt   = 0;
static float    ppgDcIr = 0, ppgDcRed = 0, ppgLpIr = 0, ppgLpRed = 0;
static float    ppgEnv = 0;
static bool     ppgArmed = true;
static uint8_t  ppgCool = 0;
static uint16_t ppgSettle = PPG_SETTLE_SAMPLES;
static uint32_t ppgT = 0, ppgLastBeatT = 0;
static bool     ppgHaveBeat = false;
static uint32_t ppgRR[4];
static uint8_t  ppgRRn = 0, ppgRRi = 0;
static uint16_t winN = 0;
static double   sumIr = 0, sumRed = 0, sumAcIr2 = 0, sumAcRed2 = 0;
static float    spo2f = 0;

// =====================================================================
//  I2C : SCANNER (diagnostik saat boot)
// =====================================================================
static void i2cScan() {
  Serial.println(F("# === I2C SCAN ==="));
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, INPUT_PULLUP);
  delay(5);
  int sda = digitalRead(PIN_SDA), scl = digitalRead(PIN_SCL);
  Serial.printf("# Level idle: SDA=%d SCL=%d\n", sda, scl);
  if (!sda || !scl) {
    Serial.println(F("# *** BUS LOW: cek kabel/pull-up, lihat README bagian Troubleshooting ***"));
  }
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);                     // 100 kHz saat scan = paling toleran
  byte found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("# FOUND 0x%02X", addr);
      if (addr == MAX_ADDR)  Serial.print(F("  <-- MAX30100"));
      if (addr == OLED_ADDR) Serial.print(F("  <-- OLED SSD1306"));
      Serial.println();
      found++;
    }
  }
  if (!found) Serial.println(F("# Tidak ada device I2C. Cek VCC, GND, SDA/SCL, pull-up."));
  Serial.println(F("# === SCAN SELESAI ==="));
}

// =====================================================================
//  I2C : DRIVER MAX30100 (register langsung)
// =====================================================================
static bool maxWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MAX_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static int maxRead(uint8_t reg) {
  Wire.beginTransmission(MAX_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom((uint8_t)MAX_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

static bool max30100Init() {
  if (maxRead(REG_PART_ID) != 0x11) return false;        // MAX30100 part ID = 0x11
  if (!maxWrite(REG_MODE, 0x40)) return false;           // soft reset
  delay(20);
  if (!maxWrite(REG_MODE, 0x03)) return false;           // mode SpO2 (IR + RED)
  // SpO2 config: hi-res (bit6) | 100 sps (001<<2) | pulse width 1600us/16-bit (11)
  if (!maxWrite(REG_SPO2_CFG, 0x40 | (0x01 << 2) | 0x03)) return false;
  // LED current: RED 27.1 mA (0x8) , IR 50 mA (0xF)
  if (!maxWrite(REG_LED_CFG, (0x8 << 4) | 0xF)) return false;
  // bersihkan FIFO
  maxWrite(REG_FIFO_WR, 0);
  maxWrite(REG_FIFO_OVF, 0);
  maxWrite(REG_FIFO_RD, 0);
  return true;
}

// Mengembalikan jumlah sampel terbaca, atau -1 jika error I2C
static int max30100ReadFifo(uint16_t* ir, uint16_t* red) {
  int wr  = maxRead(REG_FIFO_WR);
  int rd  = maxRead(REG_FIFO_RD);
  int ovf = maxRead(REG_FIFO_OVF);
  if (wr < 0 || rd < 0 || ovf < 0) return -1;

  uint8_t n = (uint8_t)((wr - rd) & 0x0F);
  if (ovf & 0x0F) n = 16;                                // FIFO penuh / overflow

  uint8_t got = 0;
  while (got < n) {
    uint8_t chunk = n - got;
    if (chunk > 8) chunk = 8;
    Wire.beginTransmission(MAX_ADDR);
    Wire.write(REG_FIFO_DATA);
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom((uint8_t)MAX_ADDR, (uint8_t)(chunk * 4)) != chunk * 4) {
      while (Wire.available()) Wire.read();
      return -1;
    }
    for (uint8_t i = 0; i < chunk; i++) {
      uint8_t a = Wire.read(), b = Wire.read(), c = Wire.read(), d = Wire.read();
      ir[got]  = ((uint16_t)a << 8) | b;                 // urutan FIFO: IR dulu, lalu RED
      red[got] = ((uint16_t)c << 8) | d;
      got++;
    }
  }
  return got;
}

// =====================================================================
//  PPG : estimasi SpO2 + pulse rate (core 0)
// =====================================================================
static void ppgReset() {
  ppgDcIr = ppgDcRed = ppgLpIr = ppgLpRed = 0;
  ppgEnv = 0; ppgArmed = true; ppgCool = 0;
  ppgSettle = PPG_SETTLE_SAMPLES;
  ppgHaveBeat = false; ppgRRn = 0; ppgRRi = 0;
  winN = 0; sumIr = sumRed = sumAcIr2 = sumAcRed2 = 0;
  spo2f = 0;
  gPpgBpm = 0; gSpo2 = 0;
}

static void ppgProcess(uint16_t ir, uint16_t red) {
  ppgT += PPG_DT_MS;                                     // waktu berbasis jumlah sampel

  // ---- deteksi jari (dengan histeresis 200 ms) ----
  if (ir < IR_FINGER_THRESH) {
    if (ppgOffCnt < 255) ppgOffCnt++;
    if (ppgOffCnt >= 20) {
      if (!gFingerOff) { gFingerOff = true; ppgReset(); }
      return;
    }
  } else {
    ppgOffCnt = 0;
    if (gFingerOff) {                                    // kontak baru
      gFingerOff = false;
      ppgReset();
      ppgDcIr = ir; ppgDcRed = red;
    }
  }

  // ---- ekstraksi komponen AC: HP (~0,5 Hz) lalu LP (~4 Hz) ----
  ppgDcIr  += ((float)ir  - ppgDcIr)  * 0.03f;
  ppgDcRed += ((float)red - ppgDcRed) * 0.03f;
  ppgLpIr  += (((float)ir  - ppgDcIr)  - ppgLpIr)  * 0.25f;
  ppgLpRed += (((float)red - ppgDcRed) - ppgLpRed) * 0.25f;
  float y = -ppgLpIr;                                    // IR turun saat sistol -> dibalik

  if (ppgSettle > 0) { ppgSettle--; return; }

  // ---- SpO2: rasio-of-rasio tiap 1 detik, R = (ACred/DCred)/(ACir/DCir) ----
  sumIr += ir; sumRed += red;
  sumAcIr2  += (double)ppgLpIr  * ppgLpIr;
  sumAcRed2 += (double)ppgLpRed * ppgLpRed;
  if (++winN >= PPG_HZ) {
    float dcIr = (float)(sumIr / winN), dcRed = (float)(sumRed / winN);
    float acIr = sqrtf((float)(sumAcIr2 / winN)), acRed = sqrtf((float)(sumAcRed2 / winN));
    if (acIr > 1.0f && acRed > 1.0f && dcIr > 0 && dcRed > 0) {
      float R = (acRed / dcRed) / (acIr / dcIr);
      float s = 110.0f - 25.0f * R;                      // rumus empiris TIDAK terkalibrasi
      if (s > 100.0f) s = 100.0f;
      if (s < 70.0f)  s = 70.0f;
      spo2f = (spo2f == 0) ? s : (0.7f * spo2f + 0.3f * s);
      gSpo2 = (uint8_t)(spo2f + 0.5f);
    }
    winN = 0; sumIr = sumRed = sumAcIr2 = sumAcRed2 = 0;
  }

  // ---- Pulse rate: ambang adaptif + refractory ----
  float a = fabsf(y);
  ppgEnv = max(ppgEnv * 0.995f, a);
  float thr = 0.55f * ppgEnv;
  if (ppgCool > 0) ppgCool--;
  if (ppgArmed && y > thr && ppgCool == 0 && ppgEnv > PPG_MIN_ENV) {
    ppgArmed = false;
    ppgCool = (uint8_t)(0.30f * PPG_HZ);
    if (ppgHaveBeat) {
      uint32_t rr = ppgT - ppgLastBeatT;
      if (rr >= 280 && rr <= 2000) {
        ppgRR[ppgRRi] = rr;
        ppgRRi = (ppgRRi + 1) & 3;
        if (ppgRRn < 4) ppgRRn++;
        uint32_t sum = 0;
        for (uint8_t i = 0; i < ppgRRn; i++) sum += ppgRR[i];
        float bpm = 60000.0f * ppgRRn / sum;
        gPpgBpm = (uint8_t)constrain((int)(bpm + 0.5f), 0, 255);
      } else {
        ppgRRn = 0; ppgRRi = 0;                          // ada detak terlewat -> mulai ulang
      }
    }
    ppgHaveBeat = true;
    ppgLastBeatT = ppgT;
    gPpgBeatCount = gPpgBeatCount + 1;
  } else if (y < 0.2f * thr) {
    ppgArmed = true;
  }
  if (ppgHaveBeat && (ppgT - ppgLastBeatT) > 4000) {     // 4 dtk tanpa detak
    gPpgBpm = 0; ppgHaveBeat = false; ppgRRn = 0; ppgRRi = 0;
  }
}

static void pollMax30100() {
  static uint8_t fails = 0;
  static uint32_t ppgIdx = 0;
  uint16_t ir[16], red[16];
  int n = max30100ReadFifo(ir, red);
  if (n < 0) {
    if (++fails > 20) {                                  // sensor hilang -> coba init ulang
      fails = 0;
      gMaxOk = false;
      gFingerOff = true;
      ppgReset();
    }
    return;
  }
  fails = 0;
  for (int i = 0; i < n; i++) {
    ppgProcess(ir[i], red[i]);
    PpgSample s = { ir[i], red[i], gFingerOff ? (uint8_t)0 : gPpgBpm, gFingerOff ? (uint8_t)0 : gSpo2, ppgIdx++ };
    xQueueSend(ppgQ, &s, 0);                             // jika penuh, sampel dibuang
  }
}

// =====================================================================
//  OLED
// =====================================================================
static void drawOled() {
  const bool blink = ((millis() / 400) & 1) != 0;
  const bool leadsOff = gLeadsOff, fingerOff = gFingerOff, danger = gDanger;
  char buf[24];

  display.clearDisplay();

  // ---- header ----
  const bool inv = danger && blink;
  if (inv) display.fillRect(0, 0, OLED_W, 9, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(inv ? SSD1306_BLACK : SSD1306_WHITE);
  display.setCursor(0, 1);
  if (leadsOff) { if (blink) display.print(F("LEADS OFF")); }
  else          display.print(F("ECG OK"));
#if ENABLE_MQTT
  if (gNetEnabled) {                                     // MQ = broker tersambung, WF = hanya WiFi
    display.setCursor(57, 1);
    display.print(gNetState == 2 ? F("MQ") : (gNetState == 1 ? F("WF") : F("--")));
  }
#endif
  if (fingerOff) strcpy(buf, "NO FINGER");
  else if (gPpgBpm > 0) snprintf(buf, sizeof(buf), "PR %u", (unsigned)gPpgBpm);
  else strcpy(buf, "PR --");
  if (!(fingerOff && !blink)) {
    display.setCursor(OLED_W - 6 * (int)strlen(buf), 1);
    display.print(buf);
  }
  display.setTextColor(SSD1306_WHITE);
  display.drawFastHLine(0, 10, OLED_W, SSD1306_WHITE);
  display.drawFastVLine(81, 11, 53, SSD1306_WHITE);

  // ---- gelombang EKG (sweep) ----
  for (uint8_t x = 0; x < WAVE_W - 1; x++) {
    uint8_t y0 = waveY[x], y1 = waveY[x + 1];
    if (y0 == 255 || y1 == 255) continue;
    display.drawLine(x, y0, x + 1, y1, SSD1306_WHITE);
  }

  // ---- panel kanan: HR & SpO2 ----
  display.setTextSize(1);
  display.setCursor(84, 12);
  display.print(F("HR"));
  if ((int32_t)(gBeatUntilMs - millis()) > 0) display.write(3);   // ikon hati (CP437)
  display.setTextSize(2);
  display.setCursor(84, 21);
  int hr = gEcgBpm;
  if (leadsOff || hr <= 0) display.print(F("--"));
  else display.print(hr);

  display.setTextSize(1);
  display.setCursor(84, 40);
  display.print(F("SpO2"));
  display.setTextSize(2);
  display.setCursor(84, 49);
  int sp = gSpo2;
  if (fingerOff || sp <= 0) {
    display.print(F("--"));
  } else {
    display.print(sp);
    display.setTextSize(1);
    display.print(F("%"));
  }
  display.display();
}

// =====================================================================
//  TASK I2C (core 0): MAX30100 + OLED
// =====================================================================
static void i2cTask(void* arg) {
  uint32_t lastDraw = 0, lastRetry = 0;
  for (;;) {
    uint32_t now = millis();
    if (!gMaxOk) {
      gFingerOff = true;
      if (now - lastRetry > 2000) {                      // coba sambung ulang tiap 2 dtk
        lastRetry = now;
        if (max30100Init()) { gMaxOk = true; ppgReset(); }
      }
    } else {
      pollMax30100();
    }
    if (gOledOk && (now - lastDraw >= OLED_PERIOD_MS)) {
      lastDraw = now;
      drawOled();
    }
    vTaskDelay(pdMS_TO_TICKS(8));
  }
}

// =====================================================================
//  EKG (core 1)
// =====================================================================
static int readEcgAdc() {
  uint32_t s = 0;
  for (uint8_t i = 0; i < 4; i++) s += analogRead(PIN_ECG);   // oversampling 4x
  return (int)(s >> 2);
}

static void pushWave(float amp) {
  float scale = ecgEnvelope * 1.4f;
  if (scale < 100.0f) scale = 100.0f;
  int y = WAVE_MID - (int)(amp / scale * WAVE_HALF);
  if (y < WAVE_TOP) y = WAVE_TOP;
  if (y > WAVE_BOT) y = WAVE_BOT;
  uint8_t i = waveIdx;
  waveY[i] = (uint8_t)y;
  for (uint8_t k = 1; k <= 4; k++) waveY[(i + k) % WAVE_W] = 255;   // batang penghapus
  waveIdx = (i + 1) % WAVE_W;
}

// Mengembalikan true pada sampel R-peak
static bool ecgProcess(int raw, bool leadsOff, uint32_t nowMs) {
  bool beat = false;
  float amp = 0;

  if (!leadsOff) {
    ecgBaseline += ((float)raw - ecgBaseline) * 0.002f;
    amp = (float)raw - ecgBaseline;
    float pos = amp > 0 ? amp : 0;
    float thr = ecgEnvelope * ECG_THRESH_FRAC;

    if (!ecgAbove) {
      if (pos > thr && (nowMs - ecgLastBeatMs) > ECG_REFRACTORY_MS) {
        ecgAbove = true;
        beat = true;
        ecgPeakAmp = pos;
        uint32_t rr = nowMs - ecgLastBeatMs;
        if (ecgHaveBeat && rr > 250 && rr < 3000) {
          float inst = 60000.0f / rr;
          ecgBpm = (ecgBpm == 0) ? (int)(inst + 0.5f)
                                 : (int)(0.6f * ecgBpm + 0.4f * inst + 0.5f);
        }
        ecgHaveBeat = true;
        ecgLastBeatMs = nowMs;
        beepUntilMs  = nowMs + BEEP_MS;
        greenUntilMs = nowMs + GREEN_MS;
        gBeatUntilMs = nowMs + 150;
      }
    } else {
      if (pos > ecgPeakAmp) ecgPeakAmp = pos;            // lacak puncak sebenarnya
      if (pos < thr * 0.5f) {                            // sinyal turun -> detak selesai
        ecgAbove = false;
        ecgEnvelope += (ecgPeakAmp - ecgEnvelope) * 0.25f;
      }
    }
    ecgEnvelope *= 0.99975f;                             // peluruhan lambat (tau ~20 dtk)
    if (ecgEnvelope < ECG_ENV_MIN) ecgEnvelope = ECG_ENV_MIN;

    if (ecgHaveBeat && (nowMs - ecgLastBeatMs) > 4000) { // 4 dtk tanpa detak
      ecgBpm = 0; ecgHaveBeat = false;
    }
  } else {
    ecgBpm = 0; ecgAbove = false; ecgHaveBeat = false;
  }

  // ---- gelombang untuk OLED: ambil sampel paling "jauh" dari baseline tiap 8 sampel ----
  if (fabsf(amp) >= fabsf(decBest)) decBest = amp;
  if (++decCount >= WAVE_DECIM) {
    pushWave(decBest);
    decBest = 0; decCount = 0;
  }
  return beat;
}

static void updateOutputs(uint32_t nowMs, bool leadsOff) {
  bool fingerOff = gFingerOff;
  int  pr = gPpgBpm, sp = gSpo2;

  bool ecgDanger = !leadsOff && ecgBpm > 0 && (ecgBpm > BPM_HIGH || ecgBpm < BPM_LOW);
  bool prBad = !fingerOff && pr > 0 && (pr > BPM_HIGH || pr < BPM_LOW);
  bool spBad = !fingerOff && sp > 0 && sp < SPO2_LOW;
  bool danger = ecgDanger || prBad || spBad;
  gDangerMask = (ecgDanger ? 1 : 0) | (prBad ? 2 : 0) | (spBad ? 4 : 0);
  gDanger = danger;

  // Biru: kedip jika ada sensor yang tidak terpasang
  if (leadsOff || fingerOff) {
    if (nowMs - blueToggleMs > BLUE_BLINK) { blueToggleMs = nowMs; blueState = !blueState; }
    digitalWrite(PIN_BLUE, blueState);
  } else {
    digitalWrite(PIN_BLUE, LOW); blueState = false;
  }
  digitalWrite(PIN_RED, danger ? HIGH : LOW);
  digitalWrite(PIN_GREEN, (!danger && nowMs < greenUntilMs) ? HIGH : LOW);
  digitalWrite(PIN_BUZZER, (danger || nowMs < beepUntilMs) ? HIGH : LOW);
}

// =====================================================================
//  JARINGAN: WiFi + MQTT  (task di core 0, hanya lewat antrean)
// =====================================================================
#if ENABLE_MQTT
#define PKT_VER         1
#define NET_BATCH_ECG   20                               // 20 sampel / 200 Hz = 100 ms per paket
#define NET_PPG_MAX     40
#define NET_PKT_MAX     (20 + 2 * NET_BATCH_ECG + 4 * NET_PPG_MAX)

struct NetPacket {
  uint32_t tMs;                                          // saat paket dibuat (untuk membuang yang basi)
  uint16_t len;
  uint8_t  data[NET_PKT_MAX];
};

#if MQTT_USE_TLS
static WiFiClientSecure netClient;
#else
static WiFiClient netClient;
#endif
static PubSubClient mqtt(netClient);
static char tStream[64], tVitals[64], tStatus[64], tAlarm[64], mqttClientId[40];

static inline void put16(uint8_t* o, uint16_t& p, uint16_t v) { o[p++] = v & 0xFF; o[p++] = v >> 8; }
static inline void put32(uint8_t* o, uint16_t& p, uint32_t v) {
  for (uint8_t i = 0; i < 4; i++) { o[p++] = v & 0xFF; v >>= 8; }
}

// Menyusun paket "stream" (format ada di komentar header). Mengembalikan panjang byte.
static uint16_t buildStreamPacket(uint8_t* o, uint8_t flags, uint32_t seq,
                                  uint32_t ecgIdx, const uint16_t* ecg, uint8_t nE,
                                  uint32_t ppgIdx, const PpgSample* ppg, uint8_t nP,
                                  uint8_t ppgBpm, uint8_t spo2, uint16_t ecgBpm) {
  uint16_t p = 0;
  o[p++] = PKT_VER;
  o[p++] = flags;
  put32(o, p, seq);
  put32(o, p, ecgIdx);
  o[p++] = nE;
  put32(o, p, ppgIdx);
  o[p++] = nP;
  o[p++] = ppgBpm;
  o[p++] = spo2;
  put16(o, p, ecgBpm);
  for (uint8_t i = 0; i < nE; i++) put16(o, p, ecg[i]);
  for (uint8_t i = 0; i < nP; i++) { put16(o, p, ppg[i].ir); put16(o, p, ppg[i].red); }
  return p;
}

// Dipanggil dari loop() (core 1). Tidak pernah memblokir: bila antrean penuh,
// paket TERLAMA dibuang agar yang tersimpan selalu data terbaru.
static void netEnqueue(const NetPacket& pk) {
  if (xQueueSend(netQ, &pk, 0) != pdTRUE) {
    NetPacket old;
    xQueueReceive(netQ, &old, 0);
    gNetDropped = gNetDropped + 1;
    xQueueSend(netQ, &pk, 0);
  }
}

static bool mqttConnect() {
  const char* u  = MQTT_USER[0] ? MQTT_USER : NULL;
  const char* pw = MQTT_USER[0] ? MQTT_PASS : NULL;
  // Last Will: bila alat mati/putus, broker otomatis menerbitkan "offline" (retained)
  if (!mqtt.connect(mqttClientId, u, pw, tStatus, 1, true, "offline")) return false;
  mqtt.publish(tStatus, "online", true);
  return true;
}

static void netTask(void* arg) {
  uint32_t lastTry = 0, lastVitals = 0;
  uint8_t  lastMask = 0xFF;
  for (;;) {
    uint32_t now = millis();

    if (WiFi.status() != WL_CONNECTED) {                 // ESP32 menyambung ulang WiFi sendiri
      gNetState = 0;
      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }
#if MQTT_USE_TLS && !MQTT_TLS_INSECURE
    if (time(nullptr) < 1700000000) {                    // verifikasi sertifikat butuh jam yang benar (NTP)
      gNetState = 1;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
#endif
    if (!mqtt.connected()) {
      gNetState = 1;
      if (now - lastTry > 2000) {
        lastTry = now;
        if (mqttConnect()) { gNetState = 2; lastVitals = 0; lastMask = 0xFF; }
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    gNetState = 2;
    mqtt.loop();

    // 1) paket gelombang; yang sudah basi (mis. setelah putus) dibuang -> tampilan tetap "live"
    NetPacket pk;
    for (uint8_t g = 0; g < 8 && xQueueReceive(netQ, &pk, 0) == pdTRUE; g++) {
      if ((uint32_t)(millis() - pk.tMs) > MQTT_MAX_AGE_MS) { gNetDropped = gNetDropped + 1; continue; }
      if (mqtt.publish(tStream, pk.data, pk.len, false)) gNetSent = gNetSent + 1;
      else { gNetDropped = gNetDropped + 1; break; }
    }

    // 2) ringkasan tanda vital 1 Hz
    if (now - lastVitals >= 1000) {
      lastVitals = now;
      char js[224];
      int n = snprintf(js, sizeof(js),
        "{\"hr\":%d,\"pr\":%u,\"spo2\":%u,\"loff\":%d,\"foff\":%d,\"danger\":%d,\"mask\":%u,"
        "\"rssi\":%d,\"up\":%lu,\"sent\":%lu,\"drop\":%lu}",
        gLeadsOff ? 0 : (int)gEcgBpm,
        (unsigned)(gFingerOff ? 0 : gPpgBpm), (unsigned)(gFingerOff ? 0 : gSpo2),
        gLeadsOff ? 1 : 0, gFingerOff ? 1 : 0, gDanger ? 1 : 0, (unsigned)gDangerMask,
        (int)WiFi.RSSI(), (unsigned long)(now / 1000),
        (unsigned long)gNetSent, (unsigned long)gNetDropped);
      mqtt.publish(tVitals, (const uint8_t*)js, n, false);
    }

    // 3) alarm: diterbitkan (retained) hanya saat statusnya berubah
    uint8_t mask = gDangerMask;
    if (mask != lastMask) {
      lastMask = mask;
      char js[48];
      int n = snprintf(js, sizeof(js), "{\"alarm\":%d,\"mask\":%u}", mask ? 1 : 0, (unsigned)mask);
      mqtt.publish(tAlarm, (const uint8_t*)js, n, true);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void netBegin() {
  if (strcmp(WIFI_SSID, "ISI_NAMA_WIFI") == 0) {
    Serial.println(F("# MQTT NONAKTIF: WIFI_SSID belum diisi (lihat KONFIGURASI JARINGAN di awal sketch)"));
    return;
  }
  snprintf(tStream, sizeof(tStream), "%s/%s/stream", MQTT_PREFIX, DEVICE_ID);
  snprintf(tVitals, sizeof(tVitals), "%s/%s/vitals", MQTT_PREFIX, DEVICE_ID);
  snprintf(tStatus, sizeof(tStatus), "%s/%s/status", MQTT_PREFIX, DEVICE_ID);
  snprintf(tAlarm,  sizeof(tAlarm),  "%s/%s/alarm",  MQTT_PREFIX, DEVICE_ID);
  snprintf(mqttClientId, sizeof(mqttClientId), "%s-%04X", DEVICE_ID,
           (unsigned)((ESP.getEfuseMac() >> 32) & 0xFFFF));       // unik per chip

#if MQTT_USE_TLS
#if MQTT_TLS_INSECURE
  netClient.setInsecure();
#else
  netClient.setCACert(MQTT_CA_CERT);
#endif
#endif

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);                                  // tanpa modem-sleep: latensi lebih stabil
  WiFi.begin(WIFI_SSID, WIFI_PASS);
#if MQTT_USE_TLS && !MQTT_TLS_INSECURE
  configTime(0, 0, "pool.ntp.org", "time.google.com");   // ESP32 tidak punya jam saat boot
#endif

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(15);

  netQ = xQueueCreate(NET_QUEUE_DEPTH, sizeof(NetPacket));
  if (netQ == NULL) { Serial.println(F("# MQTT NONAKTIF: memori antrean tidak cukup")); return; }
  gNetEnabled = true;
  xTaskCreatePinnedToCore(netTask, "net", 10240, NULL, 1, NULL, 0);
  Serial.printf("# MQTT aktif: %s:%d  id=%s  topik=%s\n", MQTT_HOST, MQTT_PORT, mqttClientId, tStream);
}
#endif   // ENABLE_MQTT

// =====================================================================
//  SETUP / LOOP
// =====================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  Serial.println();
  Serial.println(F("# ECG + SpO2 MONITOR (ESP32) booting"));

  pinMode(PIN_LO_P, INPUT);
  pinMode(PIN_LO_M, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);
  pinMode(PIN_RED, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ECG, ADC_11db);            // rentang ~0-3,3 V

  for (uint8_t i = 0; i < WAVE_W; i++) waveY[i] = 255;

  // ---- I2C: scan, lalu OLED, lalu MAX30100 ----
  i2cScan();
  gOledOk = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  Wire.setClock(400000);
  Wire.setTimeOut(20);
  if (gOledOk) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);   display.println(F("ECG + SpO2 MONITOR"));
    display.setCursor(0, 16);  display.println(F("Starting..."));
    display.display();
  } else {
    Serial.println(F("# OLED tidak ditemukan (alamat 0x3C) - lanjut tanpa OLED"));
  }

  gMaxOk = max30100Init();
  ppgReset();
  Serial.println(gMaxOk ? F("# MAX30100 OK")
                        : F("# MAX30100 TIDAK ditemukan - akan dicoba lagi otomatis"));

  if (gOledOk) {
    display.setCursor(0, 28);
    display.println(gMaxOk ? F("MAX30100: OK") : F("MAX30100: NOT FOUND"));
    display.display();
  }
  delay(800);

  ecgBaseline = readEcgAdc();                            // mulai dari DC sebenarnya

  ppgQ = xQueueCreate(64, sizeof(PpgSample));
  xTaskCreatePinnedToCore(i2cTask, "i2c", 8192, NULL, 1, NULL, 0);

#if ENABLE_MQTT
  netBegin();
#endif
}

void loop() {
  // ---- jadwal sampel EKG tepat 200 Hz ----
  static bool     started = false;
  static uint32_t nextUs = 0;
  static uint32_t lastPpgBeatSeen = 0;
#if ENABLE_MQTT
  static uint16_t   eBatch[NET_BATCH_ECG];               // penampung paket jaringan
  static uint8_t    eN = 0, pN = 0, lastNet = 255;
  static uint32_t   eIdx = 0, ecgCount = 0, netSeq = 0, ppgNext = 0;
  static PpgSample  pBatch[NET_PPG_MAX];
#endif
  if (!started) { nextUs = micros(); started = true; }

  int32_t wait = (int32_t)(nextUs - micros());
  while (wait > 0) {
    if (wait > 2000) vTaskDelay(1);                      // tidur kasar, sisanya spin
    wait = (int32_t)(nextUs - micros());
  }
  if ((int32_t)(micros() - nextUs) > (int32_t)(ECG_PERIOD_US * 4)) nextUs = micros();
  nextUs += ECG_PERIOD_US;

  uint32_t nowMs = millis();

  // ---- 1. leads-off & baca ADC ----
  bool leadsOff = (digitalRead(PIN_LO_P) == HIGH) || (digitalRead(PIN_LO_M) == HIGH);
  int  raw = leadsOff ? ADC_MID : readEcgAdc();
  gLeadsOff = leadsOff;

  // ---- 2. deteksi detak ----
  bool beat = ecgProcess(raw, leadsOff, nowMs);
  gEcgBpm = ecgBpm;

  // detak dari PPG sebagai cadangan umpan balik saat elektrode EKG lepas
  uint32_t pc = gPpgBeatCount;
  if (pc != lastPpgBeatSeen) {
    lastPpgBeatSeen = pc;
    if (leadsOff && !gFingerOff) {
      beepUntilMs = nowMs + BEEP_MS; greenUntilMs = nowMs + GREEN_MS; gBeatUntilMs = nowMs + 150;
    }
  }

  // ---- 3. LED / buzzer ----
  updateOutputs(nowMs, leadsOff);

  // ---- 4. kirim: USB serial (opsional) dan/atau paket MQTT ----
#if SERIAL_STREAM
  char b[40];
  int n = snprintf(b, sizeof(b), "E,%d,%d,%d,%d\n", raw, leadsOff ? 1 : 0, beat ? 1 : 0, ecgBpm);
  Serial.write((const uint8_t*)b, n);
#endif
  (void)beat;

  PpgSample s;
  for (uint8_t guard = 0; guard < 16 && xQueueReceive(ppgQ, &s, 0) == pdTRUE; guard++) {
#if SERIAL_STREAM
    n = snprintf(b, sizeof(b), "P,%u,%u,%u,%u\n", (unsigned)s.ir, (unsigned)s.red,
                 (unsigned)s.bpm, (unsigned)s.spo2);
    Serial.write((const uint8_t*)b, n);
#endif
#if ENABLE_MQTT
    ppgNext = s.idx + 1;
    if (pN < NET_PPG_MAX) pBatch[pN++] = s;
#endif
  }

#if ENABLE_MQTT
  if (gNetEnabled) {
    if (eN == 0) eIdx = ecgCount;
    eBatch[eN++] = (uint16_t)raw;
    ecgCount++;
    if (eN >= NET_BATCH_ECG) {                           // tiap 100 ms
      uint8_t flags = (leadsOff ? 1 : 0) | (gFingerOff ? 2 : 0) | (gDanger ? 4 : 0);
      NetPacket pk;
      pk.tMs = nowMs;
      pk.len = buildStreamPacket(pk.data, flags, netSeq++, eIdx, eBatch, eN,
                                 pN ? pBatch[0].idx : ppgNext, pBatch, pN,
                                 gFingerOff ? 0 : gPpgBpm, gFingerOff ? 0 : gSpo2,
                                 (uint16_t)(ecgBpm > 0 ? ecgBpm : 0));
      netEnqueue(pk);
      eN = 0; pN = 0;
    }
    if (gNetState != lastNet) {                          // catat perubahan status jaringan
      lastNet = gNetState;
      Serial.printf("# NET state=%d (0=WiFi putus, 1=WiFi OK, 2=MQTT OK)\n", (int)lastNet);
    }
  }
#endif
}
