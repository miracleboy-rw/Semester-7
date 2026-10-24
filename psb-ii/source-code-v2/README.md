# Monitor Pasien EKG + SpO2 — ESP32 + AD8232 + MAX30100 + OLED

Monitor tanda vital real-time dalam **satu alat**: EKG satu-lead (AD8232), oksimeter nadi/PPG (MAX30100), dan layar OLED, semuanya dikendalikan satu **ESP32**. Data dikirim lewat USB ke aplikasi Python yang menampilkan dashboard bergaya *bedside monitor* rumah sakit.

Proyek ini menggabungkan dua proyek sebelumnya (EKG Monitor berbasis Arduino dan Pulse Oximeter Monitor berbasis Arduino) menjadi satu perangkat dan satu aplikasi.

> ⚠️ **Proyek Tugas Besar.** Nilai SpO2 memakai rumus empiris tidak terkalibrasi (`110 − 25·R`). Jangan dipakai untuk keputusan kesehatan apa pun.

---

> **Baru:** alat dapat berada **jauh** dari monitor. ESP32 mengirim data lewat **WiFi ke message broker MQTT**, dan aplikasi monitor berlangganan dari broker yang sama. Lihat bab [Mode jarak jauh (WiFi + MQTT)](#mode-jarak-jauh-wifi--mqtt). Mode USB serial lama tetap berfungsi.

## Daftar Isi

1. [Fitur](#fitur)
2. [Isi folder](#isi-folder)
3. [Daftar komponen](#daftar-komponen)
4. [Panduan perakitan hardware](#panduan-perakitan-hardware)
5. [Instalasi & upload firmware](#instalasi--upload-firmware)
6. [Instalasi & menjalankan aplikasi Python](#instalasi--menjalankan-aplikasi-python)
7. [Perilaku alat: layar, LED, buzzer, alarm](#perilaku-alat-layar-led-buzzer-alarm)
8. [Mode jarak jauh (WiFi + MQTT)](#mode-jarak-jauh-wifi--mqtt)
9. [Protokol serial](#protokol-serial)
10. [Cara kerja](#cara-kerja)
11. [Troubleshooting](#troubleshooting)
12. [Perubahan dari proyek sebelumnya](#perubahan-dari-proyek-sebelumnya)

---

## Fitur

**Firmware (ESP32)**
- Sampling EKG tepat **200 Hz**, oversampling ADC 4×, deteksi R-peak adaptif, HR real-time
- Driver **MAX30100 langsung via register** (tanpa library tambahan), SpO2 + pulse rate on-device
- **OLED 128×64**: gelombang EKG sweep, HR, SpO2, PR, status leads-off / jari lepas, header alarm berkedip
- Buzzer klik per detak, LED hijau/biru/merah, alarm bahaya
- **Dual-core**: core 1 khusus sampling EKG + serial; core 0 khusus semua akses I2C (OLED + MAX30100) sehingga refresh OLED tidak mengganggu timing EKG
- I2C scanner otomatis saat boot (hasil dikirim lewat serial sebagai komentar `#`)
- Sensor MAX30100 dicoba tersambung ulang otomatis jika kabel sempat lepas
- **WiFi + MQTT (opsional):** data dikirim ke broker per 100 ms dalam paket biner ringkas; task jaringan terpisah dengan antrean, sehingga WiFi lambat/putus **tidak** mengganggu sampling EKG; *Last Will* menandai alat offline; alarm tetap lokal (buzzer/LED/OLED)

**Aplikasi Python (PyQt5 + pyqtgraph)**
- Layar hitam, grid kertas EKG, gelombang **sweep** (tidak scrolling) dengan batang penghapus dan jejak memudar
- Panel kiri 70 %: **EKG** (hijau) di atas, **pleth PPG** (cyan) di bawah. Panel kanan 30 %: **HR, SpO2, PR**, banner alarm, diagnostik
- Filter real-time: EKG → high-pass 0,5 Hz → low-pass 40 Hz → notch 50/60 Hz; PPG → band-pass 0,5–5 Hz
- Deteksi R-peak adaptif; BPM diperbarui tiap detak (waktu dihitung dari nomor sampel, bukan jam PC, jadi akurat)
- Thread serial terpisah → antrean thread-safe → GUI 60 FPS
- Deteksi leads-off dan jari lepas, reconnect serial otomatis, mode `--simulate` tanpa hardware
- **Mode `--mqtt`:** menerima dari broker, menyusun gelombang dari *indeks sampel* dengan **jitter buffer adaptif**, menandai "LINK LOST" bila data hilang dan "DEVICE OFFLINE" bila alat mati

---

## Isi folder

| File | Fungsi |
|---|---|
| `ecg_spo2_monitor.ino` | Firmware ESP32 (gabungan 2 file `.ino` lama) |
| `ecg_spo2_monitor.py` | Aplikasi dashboard desktop (gabungan 2 file `.py` lama) |
| `README.md` | Dokumen ini (gabungan 2 file `.md` lama) |
| `requirements.txt` | Dependensi Python (termasuk `paho-mqtt`) |
| `mqtt_device_sim.py` | Simulator alat: menerbitkan data sintetis ke broker (uji broker + monitor **tanpa ESP32**) |

> Di Arduino IDE, nama folder sketch **harus sama** dengan nama file `.ino`. Folder ini sudah memenuhi aturan itu, jadi cukup buka `ecg_spo2_monitor.ino`.

---

## Daftar komponen

| Komponen | Keterangan |
|---|---|
| ESP32 DevKit (ESP32-WROOM-32, 30 atau 38 pin) | Mikrokontroler utama |
| Modul AD8232 | Sensor EKG satu-lead + kabel elektrode 3 warna |
| Elektrode pad (3 buah) | Untuk RA, LA, RL |
| Modul MAX30100 (mis. GY-MAX30100) | Sensor SpO2 / pulse, I2C |
| OLED 0,96" SSD1306 128×64, **I2C** (4 pin: VCC GND SCL SDA) | Layar |
| Buzzer **aktif** 3–5 V | Bunyi detak & alarm |
| Transistor NPN (2N2222 / BC547 / S8050) + resistor 1 kΩ | Menggerakkan buzzer (disarankan) |
| 3 LED (hijau, biru, merah) + 3 resistor 220–330 Ω | Indikator |
| Resistor 2,2 kΩ – 4,7 kΩ × 2 | Hanya jika MAX30100 tidak terbaca (lihat Troubleshooting) |
| Breadboard + kabel jumper, kabel USB data | Perakitan |

---

## Panduan perakitan hardware

### 1. Tabel koneksi pin (ringkas)

Semua modul memakai **3,3 V**. **Jangan** menghubungkan apa pun ke 5 V kecuali dijelaskan di bawah.

| Komponen | Pin komponen | Pin ESP32 | Catatan |
|---|---|---|---|
| **OLED SSD1306** | VCC | **3V3** | |
| | GND | **GND** | |
| | SDA | **GPIO21** | Jalur I2C bersama |
| | SCL | **GPIO22** | Jalur I2C bersama |
| **MAX30100** | VIN | **3V3** | |
| | GND | **GND** | |
| | SDA | **GPIO21** | Jalur I2C bersama dengan OLED |
| | SCL | **GPIO22** | Jalur I2C bersama dengan OLED |
| | INT | – | Tidak dipakai, biarkan kosong |
| **AD8232** | 3.3V | **3V3** | |
| | GND | **GND** | |
| | OUTPUT | **GPIO34** | ADC1, *input-only* (cocok untuk analog) |
| | LO+ | **GPIO25** | Deteksi elektrode lepas |
| | LO− | **GPIO26** | Deteksi elektrode lepas |
| | SDN | – | Biarkan kosong |
| **Buzzer aktif** | (+) | via transistor ke **GPIO27** | Lihat bagian 3 |
| | (−) | **GND** (via transistor) | |
| **LED hijau** | anoda (+) → resistor 220–330 Ω | **GPIO18** | Kaki katoda (−) ke GND |
| **LED biru** | anoda (+) → resistor 220–330 Ω | **GPIO19** | Kaki katoda (−) ke GND |
| **LED merah** | anoda (+) → resistor 220–330 Ω | **GPIO23** | Kaki katoda (−) ke GND |

> Kaki LED yang lebih panjang adalah anoda (+).

### 2. Diagram sambungan

```
                         ┌─────────────────────────────┐
                         │          ESP32 DevKit       │
                         │                             │
   3V3 ──────────────────┤ 3V3                    GND  ├────────────── GND (semua modul)
                         │                             │
   OLED SDA ─┬───────────┤ GPIO21 (SDA)                │
   MAX30100 ─┘           │                             │
   OLED SCL ─┬───────────┤ GPIO22 (SCL)                │
   MAX30100 ─┘           │                             │
                         │                             │
   AD8232 OUTPUT ────────┤ GPIO34 (ADC1, input-only)   │
   AD8232 LO+ ───────────┤ GPIO25                      │
   AD8232 LO- ───────────┤ GPIO26                      │
                         │                             │
   Buzzer (transistor) ──┤ GPIO27                      │
   LED hijau + R ────────┤ GPIO18                      │
   LED biru  + R ────────┤ GPIO19                      │
   LED merah + R ────────┤ GPIO23                      │
                         └─────────────────────────────┘
```

**Bus I2C bersama.** OLED dan MAX30100 menempel pada **dua kabel yang sama** (SDA dan SCL). Ini aman karena alamatnya berbeda: OLED di `0x3C`, MAX30100 di `0x57`. Di breadboard cukup satukan SDA OLED, SDA MAX30100, dan GPIO21 pada baris yang sama; lakukan hal yang sama untuk SCL dengan GPIO22.

### 3. Rangkaian buzzer (disarankan pakai transistor)

Buzzer aktif sering menarik arus melebihi batas aman satu pin ESP32 (±12 mA). Pakai transistor NPN sebagai saklar:

```
              3V3 (atau VIN/5V bila buzzer 5 V)
                │
            Buzzer aktif (+)
            Buzzer aktif (−)
                │
                C
GPIO27 ──[1 kΩ]── B   NPN (2N2222 / BC547)
                E
                │
               GND
```

- Buzzer (+) ke 3V3, buzzer (−) ke kaki **Collector** transistor.
- GPIO27 → resistor 1 kΩ → kaki **Base**.
- kaki **Emitter** ke GND.
- Jika buzzer Anda bermodul (sudah ada transistor di papannya, biasanya 3 pin VCC/GND/I/O), cukup sambungkan I/O ke GPIO27, VCC ke 3V3, GND ke GND.

> Pastikan buzzer **aktif** (punya osilator sendiri, berbunyi saat diberi tegangan DC). Buzzer pasif tidak akan berbunyi dengan firmware ini.

### 4. Penempatan elektrode EKG

Kabel AD8232 yang umum: **merah = RA**, **kuning = LA**, **hijau = RL** (cek tulisan pada konektor jack kabel Anda).

| Kabel | Lokasi tempel |
|---|---|
| RA (Right Arm) | Dada kanan atas, di bawah tulang selangka kanan |
| LA (Left Arm) | Dada kiri atas, di bawah tulang selangka kiri |
| RL (Right Leg / referensi) | Perut kanan bawah / tulang rusuk kanan bawah |

Tips: bersihkan kulit (dan cukur bulu jika perlu), tempel pad dengan rapat, jangan bergerak saat pengukuran, dan jauhkan kabel dari charger/kabel listrik. Jika gelombang R tampak **terbalik**, tukar posisi RA dan LA.

### 5. Penempatan jari untuk MAX30100

Letakkan ujung jari telunjuk **ringan** (jangan ditekan keras) di atas sensor. Jaga jari tetap diam; tekanan terlalu kuat justru mengecilkan sinyal. Hindari cahaya ruangan yang sangat terang langsung ke sensor.

### 6. ⚠️ Catatan keselamatan

- Saat elektrode menempel di badan, **jangan** menghubungkan perangkat ke listrik PLN secara langsung. Pakai USB dari **laptop yang berjalan dengan baterai** (cabut charger) atau power bank.
- Jangan dipakai pada orang lain untuk keperluan diagnosis. Ini bukan perangkat medis bersertifikat.

### 7. Urutan perakitan yang disarankan

1. Pasang ESP32 di breadboard. Hubungkan rel 3V3 dan GND.
2. Sambungkan **OLED** (4 kabel) → upload firmware → pastikan layar menyala.
3. Tambahkan **MAX30100** pada jalur SDA/SCL yang sama → cek log boot terbaca `FOUND 0x57`.
4. Tambahkan **AD8232** (OUTPUT, LO+, LO−).
5. Tambahkan LED dan buzzer.
6. Tempel elektrode dan jalankan aplikasi Python.

Merakit bertahap seperti ini mempermudah mencari sumber masalah jika ada modul yang tidak bekerja.

---

## Instalasi & upload firmware

1. Pasang **Arduino IDE** (2.x disarankan).
2. Tambahkan board ESP32: *File → Preferences → Additional boards manager URLs*, isi
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`, lalu *Tools → Board → Boards Manager* → cari **esp32** (by Espressif Systems) → Install.
3. Pasang library lewat *Tools → Manage Libraries*:
   - **Adafruit SSD1306**
   - **Adafruit GFX Library** (biasanya otomatis ikut)
   - **Adafruit BusIO** (biasanya otomatis ikut)
   - **PubSubClient** oleh Nick O'Leary (hanya bila memakai MQTT, `ENABLE_MQTT 1`)

   *Tidak perlu library MAX30100*; driver sudah ada di dalam sketch.
4. Buka `ecg_spo2_monitor.ino`.
5. *Tools → Board → ESP32 Arduino → **ESP32 Dev Module***, pilih **Port** yang sesuai.
6. Klik **Upload**. Jika muncul `Connecting........`, tahan tombol **BOOT** di ESP32 sampai upload dimulai.
7. Buka *Serial Monitor* dengan baud **230400**. Anda akan melihat log boot seperti:

```
# ECG + SpO2 MONITOR (ESP32) booting
# === I2C SCAN ===
# Level idle: SDA=1 SCL=1
# FOUND 0x3C  <-- OLED SSD1306
# FOUND 0x57  <-- MAX30100
# === SCAN SELESAI ===
# MAX30100 OK
E,2051,0,0,0
...
```

> Tutup Serial Monitor sebelum menjalankan aplikasi Python (satu port hanya bisa dipakai satu program).

---

## Instalasi & menjalankan aplikasi Python

Dibutuhkan Python 3.8+.

```bash
pip install -r requirements.txt
# atau:
pip install PyQt5 pyqtgraph numpy scipy pyserial paho-mqtt
```

Menjalankan:

```bash
python ecg_spo2_monitor.py                      # auto-deteksi port ESP32
python ecg_spo2_monitor.py --port COM5          # Windows
python ecg_spo2_monitor.py --port /dev/ttyUSB0  # Linux  (macOS: /dev/cu.usbserial-xxxx)
python ecg_spo2_monitor.py --simulate           # demo tanpa hardware
python ecg_spo2_monitor.py --mqtt 192.168.1.10  # alat jauh lewat broker MQTT (lihat bab Mode jarak jauh)
```

Opsi:

| Opsi | Keterangan |
|---|---|
| `--port` | Port serial (default: auto-deteksi) |
| `--baud` | Baud rate (default **230400**, samakan dengan firmware) |
| `--mains 50\|60` | Frekuensi listrik untuk filter notch. Default **50** (Indonesia). Gunakan 60 jika Anda di wilayah 60 Hz |
| `--ir-threshold` | Level IR mentah di bawah nilai ini dianggap "tidak ada jari" (default 5000) |
| `--invert` | Balik polaritas pleth jika gelombang PPG tampil terbalik |
| `--fs-ecg`, `--fs-ppg` | Sample rate (default 200 / 100 Hz; ubah hanya jika firmware diubah) |
| `--simulate` | Data sintetis EKG + PPG tanpa hardware |
| `--mqtt HOST` | Ambil data dari broker MQTT (bukan USB). Opsi MQTT lain ada di bab Mode jarak jauh |
| `--windowed` | Mulai maximized, bukan fullscreen |

Tombol: `F11` / `F` = fullscreen, `Esc` = keluar fullscreen, `Q` = keluar.

Di Linux, jika akses port ditolak: `sudo usermod -a -G dialout $USER`, lalu logout dan login ulang.

---

## Perilaku alat: layar, LED, buzzer, alarm

### Layar OLED

```
┌────────────────────────────────┐
│ ECG OK                 PR 72   │  ← header (berkedip terbalik saat ALARM)
├────────────────────────────────┤
│                  │ HR♥         │
│  ~~/\~~~/\~~~/\  │ 72          │
│                  │ SpO2        │
│  (gelombang EKG) │ 97%         │
└────────────────────────────────┘
```

| Tampilan | Arti |
|---|---|
| `LEADS OFF` berkedip | Elektrode EKG lepas |
| `NO FINGER` berkedip | Tidak ada jari di MAX30100 |
| Header berkedip terbalik (putih) | Kondisi BAHAYA |
| `--` | Nilai belum valid (butuh beberapa detik setelah terpasang) |

### LED dan buzzer

| Indikator | Kondisi |
|---|---|
| Buzzer klik pendek | Tiap detak jantung terdeteksi |
| LED hijau berkedip | Tiap detak, saat kondisi normal |
| LED biru berkedip | Elektrode EKG lepas **atau** jari tidak ada di sensor |
| LED merah menyala + buzzer terus-menerus | **BAHAYA** |

### Batas alarm (sama di firmware dan Python)

| Parameter | Batas | Kondisi |
|---|---|---|
| HR (EKG) dan PR (PPG) | > 120 bpm | Takikardia |
| | < 50 bpm | Bradikardia |
| SpO2 | < 90 % | SpO2 rendah |

Untuk mengubahnya: `BPM_LOW`, `BPM_HIGH`, `SPO2_LOW` di `.ino` dan di bagian konfigurasi `.py`.

### Dashboard Python

- **HR (ECG)** diambil dari deteksi R-peak di Python; jika belum tersedia, memakai nilai dari firmware.
- **SpO2** dan **PR** berasal dari estimasi firmware pada data MAX30100.
- **PI** (*Perfusion Index*) menunjukkan kekuatan sinyal PPG; makin besar makin baik.
- Banner alarm menampilkan: `ECG LEADS OFF`, `SpO2 PROBE OFF`, `HR TACHY/BRADY`, `LOW SpO2`, `PULSE OUT OF RANGE`.
- Baris diagnostik bawah: sampling rate efektif EKG/PPG dan persentase paket serial rusak (*drop*).

---

## Mode jarak jauh (WiFi + MQTT)

Untuk alat yang berjauhan dengan monitor, ESP32 mengirim data lewat WiFi ke **message broker MQTT**, lalu aplikasi monitor berlangganan topik yang sama. Alat hanya membuat koneksi *keluar* ke broker, jadi **tidak perlu** *port forwarding* di lokasi alat.

```
ESP32 ──WiFi──►  Broker MQTT  ──►  Aplikasi monitor (Python)
(sampling 200 Hz   (Mosquitto, di        (jitter buffer, susun ulang
 + antrean 8 dtk)   VPS / server / LAN)   gelombang, filter, tampilan)
```

### 1. Apakah gelombang EKG tetap bagus?

Bentuk gelombang ditentukan oleh **ketepatan jarak antar-sampel**, bukan oleh kecepatan jaringan. Tiga mekanisme menjaganya:

1. **Sampling tetap di alat.** ESP32 mengambil sampel tepat 200 Hz seperti biasa; jaringan hanya mengangkut. Sampel dikumpulkan per 100 ms (20 sampel EKG) lalu dikirim sebagai satu paket.
2. **Setiap paket membawa indeks sampel.** Aplikasi menyusun gelombang dari indeks itu, bukan dari waktu tiba. BPM dihitung dari nomor sampel, jadi jitter jaringan **tidak** menggeser interval R-R.
3. **Jitter buffer adaptif** (default 0,8 detik, `--jitter-ms`) menunda tampilan sedikit supaya paket yang datang tidak teratur tetap diputar rata.

Hasil uji **simulasi** (bukan WiFi Anda): EKG 120 detik (HR 75 bpm) dikirim lewat jaringan buatan dengan jitter, lonjakan latensi, **putus 3 detik**, dan alat *reboot*. Sampel hasil susun ulang identik dengan aslinya dan HR terbaca stabil 74–75 bpm (acuan 75; selisih 1 bpm karena pembulatan interval); "LINK LOST" tampil sekitar 2 detik saat putus lalu pulih otomatis; hanya 3 kali *underrun* dalam 120 detik. Pada jaringan berjitter lebih buruk, naikkan `--jitter-ms` (mis. 1500).

> Saat koneksi benar-benar putus, gelombang **akan** terputus. Itu ditampilkan jujur sebagai `LINK LOST - NO DATA`, bukan ditutupi.

### 2. Kebutuhan data

Paket stream 100 byte (20 sampel EKG + ±10 sampel PPG) × 10 per detik, ditambah ringkasan vital 1 Hz: ±1,9 KB/detik (±15 kbps), atau ±7 MB per jam dan ±160 MB per hari (sudah termasuk header MQTT/TCP; TLS menambah ±5–10 %). Ini jauh lebih ringan dari serial USB. Bila alat memakai hotspot data seluler, perhitungkan kuota.

### 3. Langkah A: pasang broker (Mosquitto)

Broker dapat dipasang di server di LAN, VPS/cloud milik Anda, atau komputer di lokasi monitor. Untuk monitor yang **jauh lewat internet**, broker harus punya alamat yang bisa dijangkau dari kedua sisi (mis. VPS dengan IP publik/domain).

```bash
# Ubuntu / Debian
sudo apt install mosquitto mosquitto-clients
# Windows: pasang dari https://mosquitto.org/download/    Docker: image eclipse-mosquitto
```

> **Penting:** Mosquitto 2.x secara default hanya menerima koneksi dari `localhost`. Anda wajib menambahkan *listener* (di bawah).

**Uji di LAN (tanpa keamanan, hanya untuk percobaan)** — `/etc/mosquitto/conf.d/monitor.conf`:

```
listener 1883
allow_anonymous true
```

**Konfigurasi aman (disarankan, wajib bila lewat internet)** — login + ACL + TLS:

```
listener 8883
certfile /etc/letsencrypt/live/DOMAIN-ANDA/fullchain.pem
keyfile  /etc/letsencrypt/live/DOMAIN-ANDA/privkey.pem
allow_anonymous false
password_file /etc/mosquitto/passwd
acl_file /etc/mosquitto/acl
```

```bash
sudo mosquitto_passwd -c /etc/mosquitto/passwd alat      # akun untuk ESP32
sudo mosquitto_passwd    /etc/mosquitto/passwd monitor   # akun untuk aplikasi monitor
```

`/etc/mosquitto/acl` (alat hanya boleh menulis topiknya, monitor hanya membaca):

```
user alat
topic write monitor/esp32-01/#

user monitor
topic read monitor/#
```

```bash
sudo systemctl restart mosquitto
```

Buka port 1883 (atau 8883) di *firewall*/grup keamanan server. Sertifikat TLS bisa didapat gratis dari Let's Encrypt (`certbot`) bila broker punya nama domain.

### 4. Langkah B: uji broker + monitor tanpa ESP32

Pastikan broker dan aplikasi bekerja sebelum menyentuh hardware:

```bash
# terminal 1 (opsional): lihat pesan masuk
mosquitto_sub -h BROKER -u monitor -P PASSWORD -t "monitor/+/vitals" -t "monitor/+/status" -v

# terminal 2: pura-pura jadi alat
python mqtt_device_sim.py --host BROKER --user alat --password PASSWORD --faults

# terminal 3: monitor
python ecg_spo2_monitor.py --mqtt BROKER --user monitor --password PASSWORD
```

Tambahkan `--tls` bila broker memakai TLS. Simulator memakai format paket yang persis sama dengan firmware, sehingga bila langkah ini berhasil, sisi aplikasi dan broker sudah benar.

### 5. Langkah C: konfigurasi firmware

Buka bagian **KONFIGURASI JARINGAN** di awal `ecg_spo2_monitor.ino`:

| Konstanta | Arti |
|---|---|
| `ENABLE_MQTT` | `1` = aktifkan WiFi + MQTT; `0` = hanya USB serial seperti sebelumnya |
| `SERIAL_STREAM` | `1` = tetap kirim baris `E,`/`P,` ke USB; set `0` bila alat dipasang jauh tanpa PC |
| `WIFI_SSID`, `WIFI_PASS` | Jaringan WiFi (ESP32 hanya mendukung **2,4 GHz**). Selama `WIFI_SSID` masih nilai awal, MQTT tidak diaktifkan |
| `MQTT_HOST`, `MQTT_PORT` | Alamat dan port broker (1883 tanpa TLS, 8883 dengan TLS) |
| `MQTT_USER`, `MQTT_PASS` | Login broker (kosongkan bila tanpa login) |
| `MQTT_USE_TLS` | `1` = enkripsi TLS. Tempel sertifikat CA (PEM) pada `MQTT_CA_CERT` |
| `MQTT_TLS_INSECURE` | `1` = TLS tanpa verifikasi sertifikat (**hanya uji coba**) |
| `DEVICE_ID` | Id unik tiap alat (`esp32-01`, `esp32-02`, ...); membentuk topik `monitor/<id>/...` |
| `MQTT_MAX_AGE_MS` | Paket lebih tua dari ini dibuang (default 2000) |
| `NET_QUEUE_DEPTH` | Kedalaman antrean paket (default 80 = 8 detik) |

Catatan:
- Pasang library **PubSubClient** (Library Manager).
- Dengan TLS yang memverifikasi sertifikat, ESP32 otomatis menyinkronkan jam lewat NTP (`pool.ntp.org`) karena tanggal sertifikat diperiksa; butuh internet dan beberapa detik setelah WiFi tersambung. Untuk Let's Encrypt, tempel CA root yang menandatangani sertifikat broker (lihat letsencrypt.org/certificates).
- **Kurangi derau WiFi pada EKG:** lonjakan arus saat WiFi mengirim bisa mengotori sinyal AD8232. Pasang **kapasitor 100–470 µF** (elektrolit) antara 3V3 dan GND dekat ESP32/AD8232. GPIO34 aman karena memakai ADC1 (ADC2 bentrok dengan WiFi).
- Di OLED, header menampilkan status jaringan: `MQ` = tersambung ke broker, `WF` = hanya WiFi, `--` = WiFi putus. Serial Monitor mencatat perubahan `# NET state=...`.

### 6. Langkah D: jalankan monitor

```bash
python ecg_spo2_monitor.py --mqtt BROKER --device-id esp32-01 --user monitor --password PASSWORD --tls
```

| Opsi | Keterangan |
|---|---|
| `--mqtt HOST` | Alamat broker; mengaktifkan mode MQTT |
| `--mqtt-port` | Port (default 1883, atau 8883 bila `--tls`) |
| `--device-id` | Id alat, harus sama dengan `DEVICE_ID` di firmware |
| `--topic-prefix` | Awalan topik (default `monitor`) |
| `--user`, `--password` | Login broker. Password juga bisa lewat variabel lingkungan `MQTT_PASSWORD` agar tidak terlihat di riwayat perintah |
| `--tls`, `--ca-file`, `--tls-insecure` | TLS; `--ca-file` untuk broker dengan sertifikat sendiri |
| `--jitter-ms` | Tundaan jitter buffer (default 800). Naikkan bila gelombang tersendat |

Informasi tambahan di layar:
- Baris atas: `BROKER OK | DEVICE ONLINE | RSSI -61 dBm` (RSSI = kekuatan WiFi alat).
- Diagnostik: `BUF` (isi buffer), `UNDERRUN` (berapa kali buffer kosong), `SKIP` (loncat ke data terbaru).
- Banner: `LINK LOST - NO DATA` (data tidak masuk), `DEVICE OFFLINE` (alat mati/putus dari broker, lewat *Last Will*), `WAITING FOR DATA` (belum ada data sejak start).

### 7. Topik dan format data

| Topik | Isi | Catatan |
|---|---|---|
| `monitor/<id>/stream` | Paket biner gelombang, tiap 100 ms | QoS 0 |
| `monitor/<id>/vitals` | JSON 1 Hz: `hr`, `pr`, `spo2`, `loff`, `foff`, `danger`, `mask`, `rssi`, `up`, `sent`, `drop` | QoS 0 |
| `monitor/<id>/alarm` | JSON `{"alarm":0/1,"mask":n}` saat status alarm berubah | *retained* |
| `monitor/<id>/status` | `online` / `offline` | *retained*; `offline` dikirim broker lewat *Last Will* |

`mask` alarm: bit0 = HR (EKG), bit1 = PR, bit2 = SpO2 rendah.

Format paket `stream` (*little-endian*, header 20 byte):

| Offset | Tipe | Isi |
|---|---|---|
| 0 | u8 | versi (=1) |
| 1 | u8 | flags: b0 = leads-off, b1 = jari lepas, b2 = bahaya |
| 2 | u32 | nomor urut pesan (untuk mendeteksi pesan hilang) |
| 6 | u32 | indeks sampel EKG pertama |
| 10 | u8 | nE, jumlah sampel EKG (20) |
| 11 | u32 | indeks sampel PPG pertama |
| 15 | u8 | nP, jumlah sampel PPG (±10) |
| 16 | u8 | pulse rate PPG |
| 17 | u8 | SpO2 |
| 18 | u16 | HR dari EKG |
| 20 | nE × u16 | sampel EKG (12-bit) |
| … | nP × (u16, u16) | sampel PPG: IR, RED |

> Catatan desain: pustaka MQTT di ESP32 (PubSubClient) hanya menerbitkan QoS 0. Karena itu status alarm dikirim sebagai pesan *retained* **dan** ikut di `vitals` tiap detik; pesan yang terlewat otomatis terkoreksi pada detik berikutnya. MQTT berjalan di atas TCP sehingga urutan paket terjaga selama koneksi hidup.

### 8. Perilaku saat jaringan bermasalah

| Kejadian | Alat | Monitor |
|---|---|---|
| WiFi putus | Sampling, OLED, LED, buzzer **tetap jalan**; paket menumpuk di antrean (maks. 8 detik) | `LINK LOST` setelah buffer habis |
| Tersambung kembali | Paket yang sudah lebih tua dari 2 detik **dibuang**; yang terbaru dikirim | Celah kecil di gelombang, lalu kembali *live* (bukan menayangkan rekaman lama) |
| Alat mati / reboot | Broker menerbitkan `offline` (*Last Will*) | `DEVICE OFFLINE`; saat alat hidup lagi, indeks kembali ke 0 dan buffer otomatis di-reset |
| Broker mati | Alat mencoba sambung ulang tiap 2 detik | `BROKER PUTUS - SAMBUNG ULANG` |

**Alarm di alat tidak bergantung pada jaringan.** Bunyi/lampu/OLED tetap bekerja walau koneksi putus.

### 9. Keamanan

Ini data kesehatan. Bila melewati internet: wajib **TLS + user/password + ACL** (contoh di Langkah A); jangan memakai broker publik gratis (mis. `test.mosquitto.org`) untuk data asli; gunakan `DEVICE_ID` dan akun terpisah per alat; jangan memasukkan password ke repositori publik (`WIFI_PASS`, `MQTT_PASS` di sketch).

### 10. Keterbatasan

- Tampilan tertunda ±0,5–1 detik (jitter buffer + latensi jaringan): cukup untuk pemantauan, **tidak** untuk kebutuhan waktu-nyata ketat.
- Angka performa di atas berasal dari **simulasi**; ukur pada WiFi dan broker Anda sendiri (lihat `UNDERRUN`/`SKIP` dan `DROP` di diagnostik).
- Tetap proyek hobi/edukasi, **bukan alat medis**.

---

## Protokol serial

> Bagian ini untuk mode **USB serial**. Protokol paket MQTT ada di bab *Mode jarak jauh* di atas.

**230400 baud**, 1 baris ASCII per sampel, diakhiri `\n`:

```
E,<raw>,<leadsOff>,<beat>,<bpm>      EKG, 200 Hz
P,<ir>,<red>,<bpm>,<spo2>            PPG, 100 Hz
# teks bebas                         komentar/diagnostik firmware
```

| Field | Arti |
|---|---|
| `raw` | ADC 12-bit AD8232 (0–4095); datar di 2048 saat leads-off |
| `leadsOff` | 1 jika elektrode lepas |
| `beat` | 1 tepat pada sampel R-peak |
| `bpm` | HR (E) atau pulse rate (P); 0 = belum valid |
| `ir`, `red` | Nilai mentah MAX30100 16-bit |
| `spo2` | Estimasi SpO2 %; 0 = belum valid |

Baris yang tidak dikenal (misal pesan boot ESP32) diabaikan oleh parser Python.

---

## Cara kerja

1. **ESP32 core 1** membaca AD8232 tepat 200 Hz, mendeteksi R-peak (ambang adaptif 60 % dari envelope puncak), mengatur LED/buzzer, dan mengirim semua data serial.
2. **ESP32 core 0** menjalankan task I2C: membaca FIFO MAX30100 (IR + RED, 100 Hz), menghitung SpO2 (rasio-of-rasio AC/DC, `SpO2 ≈ 110 − 25·R`) dan pulse rate, serta menggambar OLED ±12 fps. Hasil PPG dikirim ke core 1 lewat antrean FreeRTOS.
3. **Thread serial Python** mem-parse baris `E` dan `P` ke dua deque terpisah.
4. **Thread GUI Python (60 FPS)** menguras deque, memfilter (EKG: HP → LP → notch; PPG: band-pass), menjalankan deteksi R-peak untuk BPM, lalu menggambar sweep.
5. **Mode MQTT:** *loop* ESP32 mengumpulkan 20 sampel EKG + sampel PPG menjadi satu paket biner dan memasukkannya ke antrean; **task jaringan di core 0** mengirimnya ke broker (paket basi dibuang). Di komputer monitor, klien MQTT meneruskan paket ke **jitter buffer** yang menyusun sampel menurut indeks, lalu sampel mengalir ke jalur filter/deteksi/tampilan yang sama seperti mode serial.

---

## Troubleshooting

**OLED tidak menyala**
- Cek VCC/GND/SDA/SCL. Lihat log boot: harus ada `FOUND 0x3C`.
- Jika alamat OLED Anda `0x3D`, ubah `OLED_ADDR` di `.ino`.
- Jika OLED Anda 1,3" dan tampil bergeser/berantakan, kemungkinan itu **SH1106**, bukan SSD1306 (butuh library lain, misalnya U8g2).

**`MAX30100 TIDAK ditemukan` / tidak ada `FOUND 0x57`**
- Cek VIN ke 3V3, GND, serta SDA→GPIO21 dan SCL→GPIO22.
- Banyak modul GY-MAX30100 menarik SDA/SCL (pull-up) ke rel internal **1,8 V**, sementara modul OLED menarik ke 3,3 V; dua tegangan yang berbeda ini bisa membuat bus "menggantung" di tegangan tanggung (±2 V) yang terbaca LOW. Diagnosis dengan multimeter: tegangan SDA ke GND saat idle sekitar 1,8–2,2 V menandakan masalah ini; mendekati 0 V berarti korsleting atau modul rusak.
- Solusi: tambahkan **pull-up eksternal 2,2 kΩ – 4,7 kΩ dari SDA dan dari SCL ke 3V3**. Pada modul yang bandel, lepas resistor pull-up SMD 1,8 V di modul MAX30100 (jika tahu caranya), atau coba modul OLED tanpa pull-up bawaan. Jika log menampilkan `BUS LOW`, cabut MAX30100 lalu reset: bila bus kembali normal, modul itulah penyebabnya.
- Firmware akan mencoba menyambung MAX30100 ulang tiap 2 detik tanpa perlu reset.

**Gelombang EKG berantakan / penuh derau**
- Pastikan pad menempel kuat dan kulit bersih. Jauhkan dari charger laptop/kabel listrik; pakai laptop mode baterai.
- Pastikan `--mains` sesuai (50 untuk Indonesia).
- Tarik napas normal dan jangan bergerak.

**HR tidak terbaca / EKG terbalik**
- Tukar posisi RA dan LA. Firmware dan Python mendeteksi R-peak **positif**.
- Pastikan OUTPUT AD8232 ke **GPIO34** (bukan pin ADC2).

**`LEADS OFF` terus**
- Cek LO+ → GPIO25 dan LO− → GPIO26, dan ketiga kabel elektrode (RA, LA, RL) terpasang.

**Pleth PPG terbalik**
- Jalankan dengan `--invert`.

**SpO2/PR tidak keluar (`--`)**
- Tunggu ±3–5 detik setelah jari terpasang. Jika tetap kosong, turunkan `--ir-threshold` atau cek posisi jari. Pada MAX30102 nilai ambang perlu lebih besar (mis. 20000).

**Python: "port tidak ditemukan" / `PORT ERROR`**
- Pasang driver USB-serial (CP210x atau CH340) sesuai chip di board Anda.
- Pakai kabel USB **data**, bukan kabel khusus charge.
- Tutup Serial Monitor Arduino.
- Tentukan port manual dengan `--port`.

**Upload gagal (`Failed to connect`)**
- Tahan tombol **BOOT** saat upload dimulai. Coba turunkan *Upload Speed* ke 115200.

**MQTT: OLED menampilkan `--` / `WF` terus, atau tidak ada data di monitor**
- `--` = WiFi belum tersambung: cek `WIFI_SSID`/`WIFI_PASS` dan pastikan jaringan **2,4 GHz**. `WF` = WiFi OK tetapi broker tidak terjangkau: cek `MQTT_HOST`, `MQTT_PORT`, *firewall*, dan bahwa Mosquitto punya *listener* (Mosquitto 2.x default hanya `localhost`).
- Lihat Serial Monitor: baris `# MQTT NONAKTIF: WIFI_SSID belum diisi` berarti konfigurasi belum diubah.
- Uji dengan `mosquitto_sub -h BROKER -t "monitor/#" -v` untuk melihat apakah pesan sampai ke broker.
- Gunakan `DEVICE_ID` yang sama di firmware dan `--device-id` di monitor.

**MQTT: monitor menampilkan `BROKER MENOLAK`**
- User/password salah atau ACL tidak mengizinkan topik. Alat butuh izin *write* ke `monitor/<id>/#`, monitor butuh izin *read*.

**MQTT + TLS: alat tidak tersambung**
- Sertifikat CA yang ditempel harus CA yang menandatangani sertifikat broker. Untuk uji, set `MQTT_TLS_INSECURE 1` sementara; bila itu berhasil, masalahnya ada di sertifikat CA atau jam (NTP).
- Pastikan alat punya akses internet ke `pool.ntp.org` (sinkronisasi jam diperlukan untuk memeriksa tanggal sertifikat).

**MQTT: gelombang tersendat / banyak `UNDERRUN`**
- Naikkan `--jitter-ms` (mis. 1500). Dekatkan alat ke *access point* (lihat `RSSI`; lebih baik dari −70 dBm). Pastikan kapasitor 100–470 µF terpasang bila EKG berderau saat WiFi aktif.

**Angka sampling di diagnostik jauh di bawah 200 / 100 SPS**
- Cek kabel USB, coba port USB lain, atau pastikan baud sama di firmware dan Python (230400).

---

## Perubahan dari proyek sebelumnya

| Aspek | Sebelumnya | Sekarang |
|---|---|---|
| Perangkat | 2 alat terpisah (Arduino Uno/Nano) | 1 alat (ESP32) |
| ADC | 10-bit (0–1023), `A0` | 12-bit (0–4095), GPIO34, oversampling 4× |
| Tegangan | 5 V | 3,3 V semua modul |
| Display | – | OLED SSD1306 I2C |
| Baud | 115200 | **230400** (dua aliran data sekaligus) |
| Protokol | `raw,leadsOff,beat,bpm` dan `ir,red,bpm,spo2` | Dibedakan awalan: `E,...` dan `P,...` |
| Library MAX30100 | MAX30100lib | Tidak perlu, driver di dalam sketch |
| I2C scanner | Sketch terpisah | Otomatis saat boot |
| Alarm | HR saja | HR, PR, dan SpO2 |
| BPM Python | Berbasis jam PC | Berbasis nomor sampel (lebih akurat) |
| Polaritas PPG | Mentah | Otomatis dibalik (IR MAX30100 turun saat sistol); `--invert` untuk membalik lagi |
| Notch | Hardcode 50 Hz | `--mains 50\|60` |
| Konektivitas | Hanya USB (alat harus dekat PC) | USB **atau** WiFi + MQTT (alat jauh) |
| Kehilangan data | Tidak ditangani (kabel dekat) | Indeks sampel, jitter buffer, "LINK LOST", "DEVICE OFFLINE" |

Perbaikan pada firmware: pelacakan envelope deteksi R-peak kini memakai **puncak sebenarnya** tiap detak (versi lama memakai nilai saat melewati ambang sehingga envelope bisa menyusut terus-menerus), dan BPM dinolkan setelah 4 detik tanpa detak agar tidak menampilkan angka basi.

---
