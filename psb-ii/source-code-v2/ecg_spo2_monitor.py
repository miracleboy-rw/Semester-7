#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
 MONITOR PASIEN GABUNGAN  --  EKG (AD8232) + SpO2/PPG (MAX30100)  |  ESP32
=============================================================================
 Dashboard gaya bedside monitor rumah sakit (PyQt5 + pyqtgraph):

   * Layar hitam, grid kertas EKG, gelombang "sweep" (tidak scrolling)
     dengan batang penghapus dan jejak memudar
   * Kiri 70 %  : gelombang EKG (hijau) di atas, pleth PPG (cyan) di bawah
   * Kanan 30 % : HR (EKG), SpO2, PR (pulse rate PPG), alarm, diagnostik
   * Filter real-time:
        EKG : high-pass 0.5 Hz -> low-pass 40 Hz -> notch 50/60 Hz
        PPG : band-pass Butterworth 0.5 - 5 Hz
   * Deteksi R-peak adaptif (mirip Pan-Tompkins ringan) -> BPM tiap detak
   * Thread terpisah untuk serial (QThread) -> deque thread-safe -> GUI 60 FPS
   * Deteksi leads-off / jari lepas, alarm berkedip, auto-reconnect serial
   * Mode --simulate tanpa hardware

 Protokol serial dari firmware ESP32 (230400 baud, ASCII, 1 baris/sampel):

     E,<raw>,<leadsOff>,<beat>,<bpm>      EKG  ~200 Hz  (raw 0..4095)
     P,<ir>,<red>,<bpm>,<spo2>            PPG  ~100 Hz  (ir/red 16-bit)
     # komentar                           diagnostik firmware (dicetak ke konsol)

 Mode MQTT (alat berada JAUH, dihubungkan lewat WiFi + message broker):
   firmware menerbitkan paket biner ke topik  monitor/<id>/stream  tiap 100 ms
   (20 sampel EKG + ~10 sampel PPG, header 20 byte, lihat decode_stream).
   Aplikasi berlangganan, menyusun ulang gelombang dari INDEKS SAMPEL (bukan
   dari waktu tiba) dan memakai jitter buffer adaptif, sehingga gelombang tetap
   mulus dan BPM tetap akurat walau jaringan tidak teratur. Data yang hilang
   ditampilkan jujur sebagai "LINK LOST", tidak disembunyikan.

 Pemakaian:
     pip install -r requirements.txt
     python ecg_spo2_monitor.py                    # auto-deteksi port ESP32
     python ecg_spo2_monitor.py --port COM5        # Windows
     python ecg_spo2_monitor.py --port /dev/ttyUSB0
     python ecg_spo2_monitor.py --simulate         # demo tanpa hardware
     python ecg_spo2_monitor.py --mqtt BROKER      # alat jauh via MQTT (id default esp32-01)
     python ecg_spo2_monitor.py --mqtt BROKER --device-id esp32-02 --tls --user monitor

 Tombol:  F11 / F = fullscreen   Esc = keluar fullscreen   Q = keluar

 CATATAN: proyek hobi/edukasi, BUKAN alat medis.
=============================================================================
"""

import sys
import time
import math
import argparse
import collections
import os
import struct
import json

import numpy as np
from scipy.signal import (butter, iirnotch, lfilter, lfilter_zi,
                          sosfilt, sosfilt_zi)

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

try:
    import serial
    from serial.tools import list_ports
except ImportError:                       # --simulate tetap jalan tanpa pyserial
    serial = None
    list_ports = None

try:
    import paho.mqtt.client as mqtt        # hanya perlu untuk mode --mqtt
except ImportError:
    mqtt = None


# ---------------------------------------------------------------------------
# KONFIGURASI
# ---------------------------------------------------------------------------
BAUD            = 230400          # harus sama dengan SERIAL_BAUD di firmware
FS_ECG          = 200.0           # Hz, harus sama dengan ECG_HZ di firmware
FS_PPG          = 100.0           # Hz, harus sama dengan PPG_HZ di firmware
WINDOW_SECONDS  = 5.0             # lebar satu sapuan layar
BLANK_FRACTION  = 0.05            # lebar batang penghapus (5 %)
GUI_FPS         = 60

ADC_MID         = 2048.0          # titik tengah ADC ESP32 12-bit
ADC_SCALE       = 1200.0          # hitungan ADC -> ~1.0 (hanya normalisasi awal)

BP_LOW, BP_HIGH = 0.5, 5.0        # band-pass PPG (Hz)
BP_ORDER        = 2
IR_FINGER_THRESH = 5000           # IR mentah di bawah ini = tidak ada jari

# MQTT (mode alat jauh)
MQTT_PREFIX     = "monitor"       # topik: <prefix>/<id>/stream | vitals | status | alarm
JITTER_MS       = 800             # tundaan tampilan (jitter buffer); naikkan bila jaringan buruk
PKT_VER         = 1
PKT_HDR         = struct.Struct("<BBIIBIBBBH")   # 20 byte, HARUS sama dengan firmware

# Batas alarm (sama dengan firmware)
BPM_LOW         = 50
BPM_HIGH        = 120
SPO2_LOW        = 90

# Palet warna
BG          = "#000000"
GREEN       = "#00FF66"
GREEN_DIM   = "#009A3E"
NEON        = "#00FFFF"
NEON_DIM    = "#00A0A8"
ALARM       = "#FF2A2A"
AMBER       = "#FFB000"
MONO = "Consolas, 'DejaVu Sans Mono', 'Courier New', monospace"


# ===========================================================================
#  1.  GENERATOR SIMULASI (murni, tanpa Qt -> mudah diuji)
# ===========================================================================
class SignalSimulator:
    """Menghasilkan sampel EKG 200 Hz + PPG 100 Hz yang sinkron."""

    def __init__(self, fs_ecg=FS_ECG, fs_ppg=FS_PPG, seed=0):
        self.fs_e, self.fs_p = fs_ecg, fs_ppg
        self.ratio = max(1, int(round(fs_ecg / fs_ppg)))
        self.t = 0.0
        self.k = 0
        self.phase = 0.0
        self.rng = np.random.default_rng(seed)
        self.mains = 50.0

    def step(self):
        """Kembalikan (ecg_tuple, ppg_tuple_atau_None) untuk satu tick EKG."""
        dt = 1.0 / self.fs_e
        t = self.t
        hr = 72 + 6 * math.sin(2 * math.pi * 0.05 * t)             # HRV ringan
        self.phase = (self.phase + dt * hr / 60.0) % 1.0
        ph = self.phase

        leads_off = (t % 60.0) >= 54.0
        finger    = (t % 45.0) < 38.0

        # ---- EKG (PQRST dari gaussian) ----
        val = (0.10 * math.exp(-((ph - 0.20) ** 2) / 0.0009)
               - 0.15 * math.exp(-((ph - 0.47) ** 2) / 0.00015)
               + 1.00 * math.exp(-((ph - 0.50) ** 2) / 0.00012)
               - 0.25 * math.exp(-((ph - 0.53) ** 2) / 0.00020)
               + 0.30 * math.exp(-((ph - 0.70) ** 2) / 0.0025))
        val += 0.05 * math.sin(2 * math.pi * self.mains * t)        # dengung listrik
        val += 0.02 * self.rng.standard_normal()                    # derau otot
        val += 0.10 * math.sin(2 * math.pi * 0.25 * t)              # baseline wander
        raw = int(max(0, min(4095, ADC_MID + val * 900)))
        if leads_off:
            ecg = (int(ADC_MID), 1, 0, 0)
        else:
            ecg = (raw, 0, 0, int(round(hr)))

        # ---- PPG (tiap ratio tick) ----
        ppg = None
        if self.k % self.ratio == 0:
            pp = (ph - 0.5) % 1.0                                   # puncak ~0.15 dtk setelah R
            wave = (math.exp(-((pp - 0.18) ** 2) / 0.006)
                    + 0.33 * math.exp(-((pp - 0.42) ** 2) / 0.012))
            resp = 0.35 * math.sin(2 * math.pi * 0.25 * t)
            if finger:
                # MAX30100 asli: IR TURUN saat sistol -> gelombang dikurangkan
                ir  = 48000 - 2600 * wave - 1800 * resp + self.rng.normal(0, 90)
                red = 39000 - 1750 * wave - 1400 * resp + self.rng.normal(0, 90)
                spo2 = 97 + 1.2 * math.sin(2 * math.pi * 0.03 * t)
                ppg = (int(ir), int(red), int(round(hr)), int(round(spo2)))
            else:
                ppg = (int(900 + self.rng.normal(0, 60)),
                       int(800 + self.rng.normal(0, 60)), 0, 0)
        self.t += dt
        self.k += 1
        return ecg, ppg


# ===========================================================================
#  2.  THREAD AKUISISI SERIAL
# ===========================================================================
class SerialReader(QtCore.QThread):
    """
    Berjalan di luar thread GUI. Tugasnya hanya: buka/buka-ulang port ->
    baca baris -> parse -> masukkan ke deque. Tanpa filter, tanpa widget.
    """
    status = QtCore.pyqtSignal(str, bool)           # (pesan, terhubung?)

    def __init__(self, q_ecg, q_ppg, port=None, baud=BAUD, simulate=False,
                 mains=50.0, parent=None):
        super().__init__(parent)
        self.q_ecg, self.q_ppg = q_ecg, q_ppg
        self.port, self.baud, self.simulate = port, baud, simulate
        self.mains = mains
        self._running = True
        self._ser = None
        # statistik untuk diagnostik
        self.n_ecg = 0
        self.n_ppg = 0
        self.dropped = 0

    # -- publik --------------------------------------------------------------
    def stop(self):
        self._running = False
        self.wait(1500)
        self._close()

    # -- internal ------------------------------------------------------------
    def _close(self):
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        self._ser = None

    @staticmethod
    def _autodetect():
        """Pilih port yang mirip ESP32 / jembatan USB-serial."""
        if list_ports is None:
            return None
        ports = list(list_ports.comports())
        keys = ("cp210", "silicon labs", "ch340", "ch910", "wch", "ftdi",
                "esp32", "usb serial", "usbserial", "usbmodem", "acm", "uart")
        for p in ports:
            blob = f"{p.description} {p.manufacturer} {p.device}".lower()
            if any(k in blob for k in keys):
                return p.device
        return ports[0].device if ports else None

    def _dispatch(self, line):
        """Parse satu baris dan taruh ke antrean yang sesuai."""
        if not line:
            return
        if line[0] == "#":                           # diagnostik firmware
            print(line)
            return
        if len(line) < 4 or line[1] != "," or line[0] not in "EP":
            return                                   # sampah boot ESP32 -> abaikan
        parts = line[2:].split(",")
        try:
            if len(parts) != 4:
                raise ValueError
            if line[0] == "E":
                raw, loff, beat, bpm = (int(p) for p in parts)
                if not (0 <= raw <= 4095 and loff in (0, 1) and beat in (0, 1)
                        and 0 <= bpm <= 300):
                    raise ValueError
                self.q_ecg.append((raw, loff, beat, bpm))
                self.n_ecg += 1
            else:
                ir, red, bpm, spo2 = (int(p) for p in parts)
                if not (0 <= ir <= 65535 and 0 <= red <= 65535
                        and 0 <= bpm <= 300 and 0 <= spo2 <= 100):
                    raise ValueError
                self.q_ppg.append((ir, red, bpm, spo2))
                self.n_ppg += 1
        except ValueError:
            self.dropped += 1                        # paket rusak -> buang

    # -- badan thread ----------------------------------------------------------
    def run(self):
        if self.simulate:
            self._run_simulation()
            return
        if serial is None:
            self.status.emit("pyserial BELUM TERPASANG", False)
            return

        backoff = 0.0
        while self._running:
            # ---------- (re)connect ----------
            if self._ser is None:
                try:
                    port = self.port or self._autodetect()
                    if port is None:
                        self.status.emit("PORT SERIAL TIDAK DITEMUKAN - MENCARI", False)
                        time.sleep(1.0)
                        continue
                    self.status.emit(f"MEMBUKA {port} @ {self.baud}", False)
                    self._ser = serial.Serial(port, self.baud, timeout=1.0)
                    time.sleep(2.0)                  # tunggu ESP32 selesai auto-reset
                    self._ser.reset_input_buffer()
                    self.status.emit(f"LINK OK  {port} @ {self.baud} BAUD", True)
                    backoff = 0.0
                except Exception as exc:
                    self._close()
                    self.status.emit(f"ERROR PORT: {type(exc).__name__} - ULANG", False)
                    backoff = min(backoff + 0.5, 3.0)
                    time.sleep(backoff)
                    continue

            # ---------- baca ----------
            try:
                raw = self._ser.readline()
                if not raw:
                    continue                          # timeout, pertahankan link
                self._dispatch(raw.decode("ascii", errors="ignore").strip())
            except (OSError, serial.SerialException) as exc:
                self._close()
                self.status.emit(f"LINK PUTUS ({type(exc).__name__}) - SAMBUNG ULANG", False)
                time.sleep(1.0)
            except Exception:
                continue

        self._close()

    def _run_simulation(self):
        self.status.emit("MODE SIMULASI - TANPA HARDWARE", True)
        sim = SignalSimulator(seed=0)
        sim.mains = self.mains
        dt = 1.0 / FS_ECG
        nxt = time.perf_counter()
        while self._running:
            now = time.perf_counter()
            if now < nxt:
                time.sleep(min(0.002, nxt - now))
                continue
            # kejar ketinggalan (timer Windows kasar) tanpa menggeser rata-rata
            guard = 0
            while nxt <= now and guard < 50:
                ecg, ppg = sim.step()
                self.q_ecg.append(ecg)
                self.n_ecg += 1
                if ppg is not None:
                    self.q_ppg.append(ppg)
                    self.n_ppg += 1
                nxt += dt
                guard += 1


# ===========================================================================
#  2b.  MQTT: DEKODER PAKET, JITTER BUFFER, KLIEN
# ===========================================================================
def decode_stream(payload):
    """
    Urai satu pesan biner topik .../stream dari firmware.
    Layout (little-endian): ver u8 | flags u8 | seq u32 | ecgIdx u32 | nE u8 |
                            ppgIdx u32 | nP u8 | ppgBpm u8 | spo2 u8 | ecgBpm u16
                            lalu nE x u16 (EKG) dan nP x (u16 IR, u16 RED).
    Mengembalikan dict, atau None bila paket rusak.
    """
    if len(payload) < PKT_HDR.size:
        return None
    (ver, flags, seq, e_idx, n_e, p_idx, n_p,
     ppg_bpm, spo2, ecg_bpm) = PKT_HDR.unpack_from(payload, 0)
    if ver != PKT_VER or len(payload) != PKT_HDR.size + 2 * n_e + 4 * n_p:
        return None
    off = PKT_HDR.size
    ecg = np.frombuffer(payload, dtype="<u2", count=n_e, offset=off).astype(np.float64)
    off += 2 * n_e
    ppg = np.frombuffer(payload, dtype="<u2", count=2 * n_p,
                        offset=off).reshape(n_p, 2).astype(np.float64)
    if (n_e and ecg.max() > 4095) or spo2 > 100 or ecg_bpm > 300:
        return None
    return {"seq": seq, "leads_off": bool(flags & 1), "finger_off": bool(flags & 2),
            "danger": bool(flags & 4), "ecg_idx": e_idx, "ecg": ecg,
            "ppg_idx": p_idx, "ppg": ppg, "ppg_bpm": ppg_bpm, "spo2": spo2,
            "ecg_bpm": ecg_bpm}


class JitterFifo:
    """
    Jitter buffer adaptif untuk satu aliran sampel (EKG atau PPG).

    * Sampel disusun berdasarkan INDEKS sampel yang dibawa paket, bukan waktu tiba.
    * Pemutaran dimulai setelah terkumpul `delay_s`, lalu laju pemutaran sedikit
      disesuaikan (maks +/-8 %) agar isi buffer tetap dekat target -> tahan terhadap
      jitter dan selisih kecil jam alat vs jam PC.
    * Celah kecil (< gap_fill_s) diisi baris `gap_row` agar sumbu waktu tetap benar;
      celah besar diberi jeda pendek lalu langsung kembali "live".
    * Bila tertinggal terlalu jauh (> max_lag_s) buffer meloncat ke data terbaru.
    """

    def __init__(self, fs, delay_s, gap_row, gap_fill_s=2.0, max_lag_s=4.0):
        self.fs = float(fs)
        self.target = max(2, int(delay_s * fs))
        self.refill = max(1, self.target // 2)
        self.need = self.target                       # ambang mulai memutar
        self.gap_fill = int(gap_fill_s * fs)
        self.max_lag = max(int(max_lag_s * fs), self.target * 3)
        self.gap_row = np.asarray(gap_row, dtype=np.float64)
        self.chunks = collections.deque()
        self.count = 0
        self.expected = None
        self.primed = False
        self.acc = 0.0
        self.underruns = 0
        self.skipped = 0

    @property
    def fill_ms(self):
        return 1000.0 * self.count / self.fs

    def _empty(self):
        return np.empty((0, self.gap_row.size))

    def _append(self, rows):
        if rows.shape[0]:
            self.chunks.append(rows)
            self.count += rows.shape[0]

    def _take(self, n):
        out, need = [], n
        while need > 0 and self.chunks:
            c = self.chunks[0]
            if c.shape[0] <= need:
                out.append(c)
                self.chunks.popleft()
                need -= c.shape[0]
            else:
                out.append(c[:need])
                self.chunks[0] = c[need:]
                need = 0
        self.count -= (n - need)
        return np.concatenate(out) if out else self._empty()

    def push(self, start_idx, rows):
        n = rows.shape[0]
        if n == 0:
            return
        if self.expected is None:
            self.expected = start_idx
        gap = start_idx - self.expected
        if gap < 0:
            if -gap > 2 * self.fs:                    # indeks mundur jauh -> alat reboot
                self.chunks.clear()
                self.count = 0
                self.primed = False
                self.need = self.target
                self.expected = start_idx
                gap = 0
            else:                                     # pesan ganda / tumpang tindih
                skip = -gap
                if skip >= n:
                    return
                rows, start_idx = rows[skip:], start_idx + skip
                n, gap = rows.shape[0], 0
        if gap > 0:
            fill = gap if gap <= self.gap_fill else int(0.5 * self.fs)
            self._append(np.tile(self.gap_row, (fill, 1)))
        self._append(rows)
        self.expected = start_idx + n
        if self.count > self.max_lag:                 # tertinggal -> loncat ke data terbaru
            self._take(self.count - self.target)
            marker = np.tile(self.gap_row, (int(0.25 * self.fs), 1))
            self.chunks.appendleft(marker)
            self.count += marker.shape[0]
            self.skipped += 1

    def release(self, dt):
        """Kembalikan baris yang harus ditampilkan untuk selang waktu dt (detik)."""
        if not self.primed:
            if self.count >= self.need:
                self.primed = True
                self.acc = 0.0
            else:
                return self._empty()
        err = (self.count - self.target) / float(self.target)
        rate = 1.0 + 0.08 * max(-1.0, min(1.0, err))
        self.acc += self.fs * dt * rate
        n = int(self.acc)
        self.acc -= n
        if n <= 0:
            return self._empty()
        if self.count < n:                            # kehabisan data -> isi ulang dulu
            out = self._take(self.count)
            self.primed = False
            self.need = self.refill
            self.acc = 0.0
            self.underruns += 1
            return out
        return self._take(n)


class MqttReader(QtCore.QThread):
    """
    Klien MQTT di thread sendiri. Berlangganan stream/vitals/status, memvalidasi
    paket, menghitung pesan yang hilang dari nomor urut, lalu menaruh paket yang
    sudah diurai ke deque. Penyusunan gelombang dilakukan di thread GUI.
    """
    status = QtCore.pyqtSignal(str, bool)

    def __init__(self, q_pkt, args, parent=None):
        super().__init__(parent)
        self.q_pkt, self.args = q_pkt, args
        base = f"{args.topic_prefix}/{args.device_id}"
        self.t_stream, self.t_vitals = base + "/stream", base + "/vitals"
        self.t_status, self.t_alarm = base + "/status", base + "/alarm"
        self._running = True
        self._client = None
        self._last_seq = None
        # statistik / info perangkat (dibaca thread GUI)
        self.n_ecg = 0                  # jumlah pesan stream diterima
        self.n_ppg = 0
        self.dropped = 0                # pesan rusak + pesan hilang (celah nomor urut)
        self.dev_online = None          # True / False / None (belum tahu)
        self.rssi = None
        self.vitals = {}

    def stop(self):
        self._running = False
        try:
            if self._client is not None:
                self._client.disconnect()
        except Exception:
            pass
        self.wait(1500)

    # -- callback paho (kompatibel paho-mqtt 1.x dan 2.x) -----------------------
    def _on_connect(self, client, userdata, flags, rc, props=None):
        failed = getattr(rc, "is_failure", None)
        failed = (rc != 0) if failed is None else failed
        if failed:
            self.status.emit(f"BROKER MENOLAK ({rc}) - CEK USER/PASSWORD/ACL", False)
            return
        client.subscribe([(self.t_stream, 0), (self.t_vitals, 0),
                          (self.t_status, 1), (self.t_alarm, 1)])
        self._last_seq = None
        self.status.emit(f"BROKER OK  {self.args.mqtt}:{self.args.mqtt_port}", True)

    def _on_disconnect(self, *a):
        self.dev_online = None
        self.status.emit("BROKER PUTUS - SAMBUNG ULANG", False)

    def _on_message(self, client, userdata, msg):
        t = msg.topic
        if t == self.t_stream:
            pkt = decode_stream(msg.payload)
            if pkt is None:
                self.dropped += 1
                return
            if self._last_seq is not None:
                gap = (pkt["seq"] - self._last_seq - 1) & 0xFFFFFFFF
                if 0 < gap < 1000:
                    self.dropped += gap
            self._last_seq = pkt["seq"]
            self.n_ecg += 1
            self.q_pkt.append(pkt)
        elif t == self.t_status:
            on = msg.payload.strip() == b"online"
            self.dev_online = on
            if on:
                self._last_seq = None
        elif t == self.t_vitals:
            try:
                self.vitals = json.loads(msg.payload.decode("utf-8", "ignore"))
                self.rssi = self.vitals.get("rssi")
            except ValueError:
                pass
        elif t == self.t_alarm:
            print("[alarm]", msg.payload.decode("utf-8", "ignore"))

    # -- badan thread --------------------------------------------------------------
    def run(self):
        if mqtt is None:
            self.status.emit("paho-mqtt BELUM TERPASANG (pip install paho-mqtt)", False)
            return
        a = self.args
        cid = f"monitor-{os.getpid()}-{int(time.time()) % 100000}"
        try:
            c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid)   # paho >= 2.0
        except AttributeError:
            c = mqtt.Client(client_id=cid)                                      # paho 1.x
        if a.user:
            c.username_pw_set(a.user, a.password)
        if a.tls:
            c.tls_set(ca_certs=a.ca_file or None)
            if a.tls_insecure:
                c.tls_insecure_set(True)
        c.on_connect, c.on_disconnect, c.on_message = (
            self._on_connect, self._on_disconnect, self._on_message)
        c.reconnect_delay_set(min_delay=1, max_delay=10)
        self._client = c
        while self._running:
            try:
                self.status.emit(f"MENYAMBUNG {a.mqtt}:{a.mqtt_port}", False)
                c.connect(a.mqtt, a.mqtt_port, keepalive=15)
                c.loop_forever(retry_first_connection=True)    # blok sampai disconnect()
            except Exception as exc:
                self.status.emit(f"ERROR BROKER: {type(exc).__name__} - ULANG", False)
                time.sleep(2.0)


# ===========================================================================
#  3.  DSP
# ===========================================================================
class ECGFilter:
    """High-pass 0.5 Hz -> low-pass 40 Hz -> notch jala-jala; stateful per blok."""

    def __init__(self, fs, mains=50.0):
        nyq = fs / 2.0
        self.hp_b, self.hp_a = butter(2, 0.5 / nyq, btype="highpass")
        self.lp_b, self.lp_a = butter(4, 40.0 / nyq, btype="lowpass")
        self.no_b, self.no_a = iirnotch(mains / nyq, Q=30.0)
        self._hp_zi0 = lfilter_zi(self.hp_b, self.hp_a)
        self.reset(0.0)

    def reset(self, x0=0.0):
        # HP diinisialisasi pada DC x0 (keluaran awal = 0); tahap lain mulai dari 0
        self.hp_zi = self._hp_zi0 * float(x0)
        self.lp_zi = lfilter_zi(self.lp_b, self.lp_a) * 0.0
        self.no_zi = lfilter_zi(self.no_b, self.no_a) * 0.0

    def process(self, x):
        x = np.asarray(x, dtype=np.float64)
        if x.size == 0:
            return x
        y, self.hp_zi = lfilter(self.hp_b, self.hp_a, x, zi=self.hp_zi)
        y, self.lp_zi = lfilter(self.lp_b, self.lp_a, y, zi=self.lp_zi)
        y, self.no_zi = lfilter(self.no_b, self.no_a, y, zi=self.no_zi)
        return y


class StreamingBandpass:
    """Band-pass Butterworth SOS stateful (untuk PPG)."""

    def __init__(self, fs, lo, hi, order=BP_ORDER):
        nyq = 0.5 * fs
        self.sos = butter(order, [lo / nyq, hi / nyq], btype="band", output="sos")
        self._zi0 = sosfilt_zi(self.sos)
        self.zi = None

    def reset(self, x0=0.0):
        self.zi = self._zi0 * float(x0)

    def process(self, x):
        x = np.asarray(x, dtype=np.float64)
        if self.zi is None:
            self.reset(x[0] if x.size else 0.0)
        y, self.zi = sosfilt(self.sos, x, zi=self.zi)
        return y


class BeatDetector:
    """
    Detektor R-peak adaptif: ambang = rata-rata + K * simpangan baku sinyal
    sendiri. Waktu dihitung dari NOMOR SAMPEL (bukan jam PC) sehingga interval
    R-R akurat walau sampel datang berkelompok per frame.
    """

    def __init__(self, fs):
        self.fs = fs
        self.alpha = 1.0 / (0.6 * fs)
        self.K = 3.5
        self.refractory = 0.28
        self.reset()

    def reset(self):
        self.mu = 0.0
        self.var = 0.02
        self.last_peak_t = None
        self.armed = True
        self.bpm = 0
        self.warm = int(1.0 * self.fs)      # abaikan 1 dtk pertama setelah reset

    def update(self, sample, t):
        d = sample - self.mu
        self.mu += self.alpha * d
        self.var += self.alpha * (d * d - self.var)
        std = max(self.var ** 0.5, 1e-4)
        thr = self.mu + self.K * std

        if self.last_peak_t is not None and (t - self.last_peak_t) > 4.0:
            self.bpm = 0                    # 4 dtk tanpa detak -> BPM basi
            self.last_peak_t = None

        if self.warm > 0:
            self.warm -= 1
            return False

        beat = False
        if self.armed and sample > thr:
            if self.last_peak_t is None or (t - self.last_peak_t) > self.refractory:
                if self.last_peak_t is not None:
                    rr = t - self.last_peak_t
                    if 0.25 < rr < 3.0:
                        inst = 60.0 / rr
                        self.bpm = int(round(inst if self.bpm == 0
                                             else 0.6 * self.bpm + 0.4 * inst))
                self.last_peak_t = t
                self.armed = False
                beat = True
        if sample < self.mu + 0.5 * std:
            self.armed = True
        return beat


class PpgBeatFlagger:
    """Penanda puncak sederhana, hanya untuk ikon hati berkedip."""

    def __init__(self, fs):
        self.refractory = int(0.30 * fs)
        self.cool = 0
        self.armed = True
        self.env = 1e-6

    def feed(self, y):
        beat = False
        for v in y:
            self.env = max(0.995 * self.env, abs(v))
            thr = 0.55 * self.env
            if self.cool > 0:
                self.cool -= 1
            if self.armed and v > thr and self.cool == 0:
                beat, self.armed, self.cool = True, False, self.refractory
            elif v < 0.2 * thr:
                self.armed = True
        return beat


# ===========================================================================
#  4.  KOMPONEN UI
# ===========================================================================
class VitalCard(QtWidgets.QFrame):
    """Satu pembacaan angka besar: judul, nilai, satuan, status alarm."""

    def __init__(self, title, unit, color=NEON, dim=NEON_DIM, value_pt=64, parent=None):
        super().__init__(parent)
        self.base_color, self.dim_color, self.unit_txt = color, dim, unit
        self._last_val = None
        self._last_unit = None
        self.setStyleSheet(f"background:{BG}; border:none;")

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(18, 2, 18, 2)
        lay.setSpacing(0)

        top = QtWidgets.QHBoxLayout()
        self.title = QtWidgets.QLabel(title)
        self.title.setStyleSheet(
            f"color:{color}; font-family:{MONO}; font-size:22px;"
            "font-weight:bold; letter-spacing:3px;")
        self.unit = QtWidgets.QLabel(unit)
        self.unit.setStyleSheet(f"color:{dim}; font-family:{MONO}; font-size:18px;")
        top.addWidget(self.title)
        top.addStretch(1)
        top.addWidget(self.unit)
        lay.addLayout(top)

        self.value = QtWidgets.QLabel("--")
        f = QtGui.QFont("Consolas")
        f.setStyleHint(QtGui.QFont.Monospace)
        f.setPointSize(value_pt)
        f.setBold(True)
        self.value.setFont(f)
        self.value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.value.setStyleSheet(f"color:{color};")
        lay.addWidget(self.value, 1)

        self.sub = QtWidgets.QLabel("")
        self.sub.setAlignment(QtCore.Qt.AlignRight)
        self.sub.setStyleSheet(
            f"color:{dim}; font-family:{MONO}; font-size:14px; letter-spacing:2px;")
        lay.addWidget(self.sub)

    def set_value(self, text, alarm=False, dim=False):
        col = ALARM if alarm else (self.dim_color if dim else self.base_color)
        key = (text, col)
        if key == self._last_val:                    # hindari restyle tiap frame
            return
        self._last_val = key
        self.value.setText(text)
        self.value.setStyleSheet(f"color:{col};")

    def set_beat(self, on):
        txt = f"{self.unit_txt} \u2665" if on else self.unit_txt
        if txt == self._last_unit:
            return
        self._last_unit = txt
        self.unit.setText(txt)
        self.unit.setStyleSheet(
            f"color:{self.base_color if on else self.dim_color};"
            f"font-family:{MONO}; font-size:18px;")


class Separator(QtWidgets.QFrame):
    def __init__(self):
        super().__init__()
        self.setFixedHeight(1)
        self.setStyleSheet(f"background:{NEON_DIM};")


class SweepTrace:
    """Satu plot sweep: buffer cincin, kurva badan/jejak/kepala, batang penghapus, grid."""

    def __init__(self, plot_widget, fs, seconds, y_lo, y_hi, rgb, caption):
        self.fs = fs
        self.N = int(fs * seconds)
        self.blank = max(4, int(self.N * BLANK_FRACTION))
        self.trail = max(8, int(self.N * 0.11))
        self.x = np.arange(self.N, dtype=np.float64)
        self.buf = np.full(self.N, np.nan)
        self.wr = 0
        r, g, b = rgb

        pi = plot_widget.getPlotItem()
        pi.hideButtons()
        pi.setMenuEnabled(False)
        pi.setMouseEnabled(x=False, y=False)
        pi.hideAxis("left")
        pi.hideAxis("bottom")
        pi.setXRange(0, self.N, padding=0)
        pi.setYRange(y_lo, y_hi, padding=0)
        plot_widget.setStyleSheet("border:none;")

        # grid kertas medis: minor 0.2 dtk / 0.2 unit, mayor 1 dtk / 1 unit
        minor = pg.mkPen((r, g, b, 26), width=1)
        major = pg.mkPen((r, g, b, 54), width=1)
        step = self.N / (seconds * 5.0)
        for i in range(int(seconds * 5) + 1):
            pi.addItem(pg.InfiniteLine(pos=i * step, angle=90,
                                       pen=major if i % 5 == 0 else minor),
                       ignoreBounds=True)
        for j in range(int(math.ceil(y_lo / 0.2)), int(math.floor(y_hi / 0.2)) + 1):
            pi.addItem(pg.InfiniteLine(pos=j * 0.2, angle=0,
                                       pen=major if j % 5 == 0 else minor),
                       ignoreBounds=True)

        self.body = pi.plot(pen=pg.mkPen(QtGui.QColor(r, g, b, 150), width=1.6))
        self.mid = pi.plot(pen=pg.mkPen(QtGui.QColor(r, g, b, 205), width=2.4))
        self.head = pi.plot(pen=pg.mkPen(
            QtGui.QColor(min(255, r + 190), min(255, g + 190), min(255, b + 190), 255),
            width=3.2))
        for c in (self.body, self.mid, self.head):
            c.setData(self.x, self.buf, connect="finite")

        self.dot = pg.ScatterPlotItem(size=9, brush=pg.mkBrush(220, 255, 255, 255),
                                      pen=pg.mkPen(r, g, b, 255))
        pi.addItem(self.dot)

        self.eraser = pg.LinearRegionItem(values=[0, self.blank], movable=False,
                                          brush=pg.mkBrush(0, 0, 0, 235))
        self.eraser.setZValue(20)
        for ln in self.eraser.lines:
            ln.setPen(pg.mkPen(r, g, b, 40))
        pi.addItem(self.eraser)

        cap = pg.TextItem(caption, color=(r, g, b, 170), anchor=(0, 0))
        cap.setPos(6, y_hi - 0.04)
        cap.setFont(QtGui.QFont("Consolas", 13, QtGui.QFont.Bold))
        pi.addItem(cap)

    def write(self, y):
        n = y.size
        if n == 0:
            return
        if n >= self.N:
            y, n = y[-self.N:], self.N
        end = self.wr + n
        if end <= self.N:
            self.buf[self.wr:end] = y
        else:
            k = self.N - self.wr
            self.buf[self.wr:] = y[:k]
            self.buf[:end - self.N] = y[k:]
        self.wr = end % self.N

    def refresh(self):
        buf, w = self.buf, self.wr
        buf[(np.arange(w, w + self.blank) % self.N)] = np.nan     # batang penghapus

        self.body.setData(self.x, buf, connect="finite")
        lo = max(0, w - self.trail)
        self.mid.setData(self.x[lo:w], buf[lo:w], connect="finite")
        lo2 = max(0, w - self.trail // 4)
        self.head.setData(self.x[lo2:w], buf[lo2:w], connect="finite")

        if w > 0 and np.isfinite(buf[w - 1]):
            self.dot.setData([w - 1], [buf[w - 1]])
        else:
            self.dot.setData([], [])
        self.eraser.setRegion([w, min(w + self.blank, self.N)])


# ===========================================================================
#  5.  JENDELA UTAMA
# ===========================================================================
class MonitorWindow(QtWidgets.QMainWindow):

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.fs_e, self.fs_p = args.fs_ecg, args.fs_ppg

        # ---- antrean bersama (deque = thread-safe untuk append/popleft) ----
        self.q_ecg = collections.deque(maxlen=20000)
        self.q_ppg = collections.deque(maxlen=10000)
        self.q_pkt = collections.deque(maxlen=2000)      # paket MQTT mentah (mode --mqtt)
        self.use_mqtt = bool(getattr(args, "mqtt", None))

        # ---- state EKG ----
        self.ecg_filt = ECGFilter(self.fs_e, args.mains)
        self.ecg_det = BeatDetector(self.fs_e)
        self.ecg_n = 0                       # nomor sampel (basis waktu detektor)
        self.ecg_need_reset = True
        self.leads_off = True
        self.sig_loss = True                 # data tidak masuk (kabel/jaringan)
        self.hw_bpm = 0
        self.env_e = 0.5
        self.gain_e = 1.0
        self.last_rx_e = 0.0
        self.beat_e = 0

        # ---- state PPG ----
        self.ppg_bp = StreamingBandpass(self.fs_p, BP_LOW, BP_HIGH)
        self.ppg_flag = PpgBeatFlagger(self.fs_p)
        self.finger = False
        self.ppg_loss = True
        self.pr = 0.0
        self.spo2 = 0.0
        self.ir_ema = 0.0
        self.gain_p = 1.0
        self.env_p = 0.0
        self.pi_pct = 0.0
        self.last_rx_p = 0.0
        self.beat_p = 0

        # ---- diagnostik ----
        self.cnt_e = self.cnt_p = 0
        self.sps_e = self.sps_p = 0.0
        self._t0 = time.perf_counter()
        self._last_tick = time.perf_counter()
        self._blink = True
        self._link_msg, self._link_ok, self._link_last = "STARTING...", False, None

        self._build_ui()

        # ---- thread akuisisi ----
        if self.use_mqtt:
            delay = args.jitter_ms / 1000.0
            self.jb_e = JitterFifo(self.fs_e, delay, [ADC_MID, 2, 0, 0])   # 2 = data hilang
            self.jb_p = JitterFifo(self.fs_p, delay, [-1, -1, 0, 0])       # -1 = data hilang
            self.reader = MqttReader(self.q_pkt, args)
            self.setWindowTitle("PATIENT MONITOR  --  ECG + SpO2  (MQTT)")
        else:
            self.reader = SerialReader(self.q_ecg, self.q_ppg, args.port, args.baud,
                                       args.simulate, args.mains)
        self.reader.status.connect(self._on_status)
        self.reader.start()

        # ---- timer ----
        self.timer = QtCore.QTimer(self)
        self.timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.timer.timeout.connect(self._tick)
        self.timer.start(int(1000 / GUI_FPS))

        self.blinker = QtCore.QTimer(self)
        self.blinker.timeout.connect(self._blink_tick)
        self.blinker.start(450)

        self.clock = QtCore.QTimer(self)
        self.clock.timeout.connect(self._clock_tick)
        self.clock.start(1000)

    # -----------------------------------------------------------------------
    #  UI
    # -----------------------------------------------------------------------
    def _build_ui(self):
        self.setWindowTitle("PATIENT MONITOR  --  ECG + SpO2")
        self.resize(1280, 720)
        self.setMinimumSize(1024, 600)
        self.setStyleSheet(f"background:{BG};")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ---------- strip status atas ----------
        strip = QtWidgets.QWidget()
        strip.setFixedHeight(34)
        sl = QtWidgets.QHBoxLayout(strip)
        sl.setContentsMargins(16, 0, 16, 0)
        css = f"color:{NEON_DIM}; font-family:{MONO}; font-size:15px; letter-spacing:2px;"
        self.lbl_link = QtWidgets.QLabel("STARTING...")
        self.lbl_link.setStyleSheet(css)
        self.lbl_mode = QtWidgets.QLabel(
            f"ECG II 0.5-40Hz + NOTCH {int(self.args.mains)}Hz   |   PLETH 0.5-5Hz")
        self.lbl_mode.setStyleSheet(css)
        self.lbl_clock = QtWidgets.QLabel("--:--:--")
        self.lbl_clock.setStyleSheet(
            f"color:{NEON}; font-family:{MONO}; font-size:15px; letter-spacing:2px;")
        sl.addWidget(self.lbl_link)
        sl.addStretch(1)
        sl.addWidget(self.lbl_mode)
        sl.addStretch(1)
        sl.addWidget(self.lbl_clock)
        root.addWidget(strip)

        # ---------- badan 70 / 30 ----------
        body = QtWidgets.QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        root.addLayout(body, 1)
        body.addLayout(self._build_plots(), 70)
        body.addWidget(self._build_panel(), 30)

    def _build_plots(self):
        pg.setConfigOptions(antialias=True, background=BG, foreground=NEON)
        col = QtWidgets.QVBoxLayout()
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(0)

        self.plot_ecg = pg.PlotWidget()
        self.plot_ppg = pg.PlotWidget()
        self.tr_ecg = SweepTrace(self.plot_ecg, self.fs_e, WINDOW_SECONDS,
                                 -1.2, 1.6, (0, 255, 102), "ECG  II")
        self.tr_ppg = SweepTrace(self.plot_ppg, self.fs_p, WINDOW_SECONDS,
                                 -1.15, 1.15, (0, 255, 255), "PLETH")
        col.addWidget(self.plot_ecg, 3)
        col.addWidget(self.plot_ppg, 2)
        return col

    def _build_panel(self):
        panel = QtWidgets.QWidget()
        panel.setStyleSheet(f"background:{BG}; border-left:1px solid {NEON_DIM};")
        lay = QtWidgets.QVBoxLayout(panel)
        lay.setContentsMargins(0, 6, 0, 6)
        lay.setSpacing(4)

        self.card_hr = VitalCard("HR  (ECG)", "bpm", GREEN, GREEN_DIM)
        self.card_spo2 = VitalCard("SpO2", "%", NEON, NEON_DIM)
        self.card_pr = VitalCard("PR  (PLETH)", "bpm", NEON, NEON_DIM)
        self.card_hr.sub.setText(f"LIMIT  {BPM_LOW}-{BPM_HIGH}")
        self.card_spo2.sub.setText(f"LIMIT  >{SPO2_LOW}")
        self.card_pr.sub.setText(f"LIMIT  {BPM_LOW}-{BPM_HIGH}")

        lay.addWidget(self.card_hr, 1)
        lay.addWidget(Separator())
        lay.addWidget(self.card_spo2, 1)
        lay.addWidget(Separator())
        lay.addWidget(self.card_pr, 1)
        lay.addWidget(Separator())

        self.lbl_pi = QtWidgets.QLabel("PI  --")
        self.lbl_pi.setStyleSheet(
            f"color:{NEON_DIM}; font-family:{MONO}; font-size:15px;"
            "letter-spacing:2px; padding-left:18px;")
        lay.addWidget(self.lbl_pi)

        self._alarm_txt = None
        self.lbl_alarm = QtWidgets.QLabel("")
        self.lbl_alarm.setWordWrap(True)
        self.lbl_alarm.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_alarm.setStyleSheet(
            f"color:{ALARM}; font-family:{MONO}; font-size:18px;"
            "font-weight:bold; letter-spacing:2px; padding:4px;")
        self.lbl_alarm.setFixedHeight(84)
        lay.addWidget(self.lbl_alarm)

        self.lbl_diag = QtWidgets.QLabel("ECG -- SPS | PPG -- SPS | DROP --%")
        self.lbl_diag.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_diag.setStyleSheet(
            f"color:{NEON_DIM}; font-family:{MONO}; font-size:12px; letter-spacing:1px;")
        lay.addWidget(self.lbl_diag)
        return panel

    # -----------------------------------------------------------------------
    #  PEMROSESAN (thread GUI)
    # -----------------------------------------------------------------------
    @staticmethod
    def _pop_all(q, limit=4000):
        out = []
        for _ in range(limit):
            try:
                out.append(q.popleft())
            except IndexError:
                break
        return out

    def _pump_mqtt(self, dt):
        """Mode MQTT: paket -> jitter buffer -> antrean sampel (sama seperti mode serial)."""
        for p in self._pop_all(self.q_pkt, limit=500):
            n_e = p["ecg"].size
            if n_e:
                rows = np.zeros((n_e, 4))               # [raw, flag, beat, bpm]
                rows[:, 0] = p["ecg"]
                rows[:, 1] = 1.0 if p["leads_off"] else 0.0
                rows[:, 3] = p["ecg_bpm"]
                self.jb_e.push(p["ecg_idx"], rows)
            n_p = p["ppg"].shape[0]
            if n_p:
                rows = np.zeros((n_p, 4))               # [ir, red, bpm, spo2]
                rows[:, 0:2] = p["ppg"]
                rows[:, 2] = p["ppg_bpm"]
                rows[:, 3] = p["spo2"]
                self.jb_p.push(p["ppg_idx"], rows)
        self.q_ecg.extend(self.jb_e.release(dt).tolist())
        self.q_ppg.extend(self.jb_p.release(dt).tolist())

    def _process_ecg(self):
        samples = self._pop_all(self.q_ecg)
        now = time.perf_counter()

        if samples:
            self.last_rx_e = now
            self.cnt_e += len(samples)
            arr = np.asarray(samples, dtype=np.float64)
            flag = arr[:, 1]
            self.leads_off = bool(flag[-1] == 1)        # 1 = elektrode lepas
            self.sig_loss = bool(flag[-1] == 2)         # 2 = data hilang (jaringan)
            self.hw_bpm = int(arr[-1, 3])

            bad = np.flatnonzero(flag != 0)
            k = int(bad[-1]) + 1 if bad.size else 0     # sampel [0:k) tidak valid
            parts = []
            if k:
                self.ecg_need_reset = True
                self.ecg_det.reset()
                self.ecg_n += k                         # jaga basis waktu detektor
                parts.append(np.zeros(k))               # garis datar
            good = arr[k:, 0]
            if good.size:
                x = (good - ADC_MID) / ADC_SCALE
                if self.ecg_need_reset:                 # kontak baru / start / selesai gap
                    self.ecg_filt.reset(x[0])
                    self.ecg_det.reset()
                    self.env_e, self.gain_e = 0.5, 1.0
                    self.ecg_need_reset = False
                y = self.ecg_filt.process(x)
                for s_ in y:
                    t = self.ecg_n / self.fs_e
                    self.ecg_n += 1
                    if self.ecg_det.update(float(s_), t):
                        self.beat_e = 9
                # auto-gain: puncak meluruh pelan agar R-wave mengisi layar
                peak = float(np.max(np.abs(y)))
                self.env_e = max(peak, self.env_e * (0.9995 ** y.size))
                target = min(0.9 / max(self.env_e, 0.05), 8.0)
                self.gain_e += 0.1 * (target - self.gain_e)
                parts.append(np.clip(y * self.gain_e, -1.15, 1.55))
            self.tr_ecg.write(np.concatenate(parts))

        elif self.last_rx_e and (now - self.last_rx_e) > 1.0:    # tidak ada data masuk
            self.sig_loss = True
            self.ecg_need_reset = True
            self.ecg_det.reset()
            self.tr_ecg.write(np.zeros(max(1, int(self.fs_e / GUI_FPS))))

    def _process_ppg(self):
        samples = self._pop_all(self.q_ppg)
        now = time.perf_counter()

        if samples:
            self.last_rx_p = now
            self.cnt_p += len(samples)
            arr = np.asarray(samples, dtype=np.float64)
            lost = arr[:, 0] < 0                        # -1 = data hilang (jaringan)
            self.ppg_loss = bool(lost[-1])
            k = int(np.flatnonzero(lost)[-1]) + 1 if lost.any() else 0
            parts = []
            if k:                                       # mulai ulang deteksi jari setelah gap
                self.ir_ema = 0.0
                self.finger = False
                self.pr, self.spo2, self.pi_pct = 0.0, 0.0, 0.0
                parts.append(np.zeros(k))
            arr = arr[k:]

            if arr.shape[0]:
                ir = arr[:, 0]
                for b_, s_ in zip(arr[:, 2], arr[:, 3]):
                    if b_ > 0:
                        self.pr = b_
                    if s_ > 0:
                        self.spo2 = s_

                a_ = 0.15                               # EMA level DC IR -> deteksi jari
                for v in (ir[0], ir[-1]):
                    self.ir_ema = (1 - a_) * self.ir_ema + a_ * v if self.ir_ema else v
                present = self.ir_ema > self.args.ir_threshold

                if present:
                    if not self.finger:                 # kontak baru
                        self.ppg_bp.reset(ir[0])
                        self.gain_p, self.env_p = 1.0, 0.0
                        self.pr, self.spo2 = (arr[-1, 2], arr[-1, 3])
                    y = self.ppg_bp.process(ir)
                    # MAX30100: IR TURUN saat sistol -> dibalik agar puncak ke atas.
                    # --invert membalik lagi bila sensor Anda tampil terbalik.
                    y = y if self.args.invert else -y

                    peak = float(np.max(np.abs(y))) if y.size else 0.0
                    self.env_p = max(peak, self.env_p * 0.997)
                    if self.env_p > 1e-6:
                        self.gain_p += 0.05 * (0.90 / self.env_p - self.gain_p)
                    yn = np.clip(y * self.gain_p, -1.1, 1.1)
                    if self.ppg_flag.feed(y):
                        self.beat_p = 9
                    self.pi_pct = 100.0 * 2.0 * self.env_p / max(self.ir_ema, 1.0)
                else:
                    self.ppg_bp.reset(self.ir_ema)
                    yn = np.zeros_like(ir)
                    self.pi_pct = 0.0
                    self.pr, self.spo2 = 0.0, 0.0
                self.finger = present
                parts.append(yn)
            self.tr_ppg.write(np.concatenate(parts))

        elif self.last_rx_p and (now - self.last_rx_p) > 1.0:
            self.ppg_loss = True
            self.finger = False
            self.tr_ppg.write(np.zeros(max(1, int(self.fs_p / GUI_FPS))))

    def _tick(self):
        now = time.perf_counter()
        dt = min(now - self._last_tick, 0.25)
        self._last_tick = now
        if self.use_mqtt:
            self._pump_mqtt(dt)
        self._process_ecg()
        self._process_ppg()
        self.tr_ecg.refresh()
        self.tr_ppg.refresh()
        self._refresh_numbers()

    # -----------------------------------------------------------------------
    def _refresh_numbers(self):
        blink = self._blink

        # ---- HR dari EKG (utamakan Python, cadangan nilai firmware) ----
        hr = 0 if (self.leads_off or self.sig_loss) else (self.ecg_det.bpm or self.hw_bpm)
        hr_alarm = hr > 0 and not (BPM_LOW <= hr <= BPM_HIGH)
        self.card_hr.set_value(str(hr) if hr else "--",
                               alarm=hr_alarm and blink, dim=hr == 0)

        # ---- SpO2 & PR dari PPG ----
        s = int(round(self.spo2)) if (self.finger and self.spo2 > 0) else 0
        p = int(round(self.pr)) if (self.finger and self.pr > 0) else 0
        s_alarm = s > 0 and s < SPO2_LOW
        p_alarm = p > 0 and not (BPM_LOW <= p <= BPM_HIGH)
        self.card_spo2.set_value(str(s) if s else "--", alarm=s_alarm and blink, dim=s == 0)
        self.card_pr.set_value(str(p) if p else "--", alarm=p_alarm and blink, dim=p == 0)

        # ikon hati berkedip
        if self.beat_e > 0:
            self.beat_e -= 1
        if self.beat_p > 0:
            self.beat_p -= 1
        self.card_hr.set_beat(self.beat_e > 0 and not (self.leads_off or self.sig_loss))
        self.card_pr.set_beat(self.beat_p > 0 and self.finger)

        self.lbl_pi.setText(f"PI  {self.pi_pct:4.1f} %" if self.finger else "PI  --")

        # ---- banner alarm ----
        lines = []
        if self.sig_loss or self.ppg_loss:               # tidak ada data: kabel/jaringan
            lines.append("WAITING FOR DATA" if not (self.last_rx_e or self.last_rx_p)
                         else "LINK LOST - NO DATA")
            if self.use_mqtt and self.reader.dev_online is False:
                lines.append("DEVICE OFFLINE")
        else:
            if self.leads_off:
                lines.append("ECG LEADS OFF")
            if not self.finger:
                lines.append("SpO2 PROBE OFF")
            if hr_alarm:
                lines.append("HR TACHY" if hr > BPM_HIGH else "HR BRADY")
            if s_alarm:
                lines.append("LOW SpO2")
            if p_alarm:
                lines.append("PULSE OUT OF RANGE")
        txt = "\n".join(lines) if (lines and blink) else ""
        if txt != self._alarm_txt:
            self._alarm_txt = txt
            self.lbl_alarm.setText(txt)

    # -----------------------------------------------------------------------
    def _blink_tick(self):
        self._blink = not self._blink

    def _clock_tick(self):
        self.lbl_clock.setText(time.strftime("%H:%M:%S"))
        now = time.perf_counter()
        dt = max(now - self._t0, 1e-3)
        self.sps_e, self.sps_p = self.cnt_e / dt, self.cnt_p / dt
        self.cnt_e = self.cnt_p = 0
        self._t0 = now
        r = self.reader
        tot = r.n_ecg + r.n_ppg + r.dropped
        drop = 100.0 * r.dropped / tot if tot else 0.0
        txt = f"ECG {self.sps_e:5.1f} SPS | PPG {self.sps_p:5.1f} SPS | DROP {drop:.1f}%"
        if self.use_mqtt:
            txt += (f"\nBUF {self.jb_e.fill_ms:4.0f} ms | UNDERRUN {self.jb_e.underruns}"
                    f" | SKIP {self.jb_e.skipped}")
        self.lbl_diag.setText(txt)
        self._update_link()

    def _on_status(self, msg, ok):
        self._link_msg, self._link_ok = msg, ok
        self._update_link()

    def _update_link(self):
        msg = self._link_msg
        if self.use_mqtt and self._link_ok:              # tambah info perangkat
            r = self.reader
            dev = {True: "DEVICE ONLINE", False: "DEVICE OFFLINE", None: "DEVICE ?"}[r.dev_online]
            msg += f"  |  {dev}"
            if r.rssi is not None:
                msg += f"  |  RSSI {r.rssi} dBm"
        key = (msg, self._link_ok)
        if key == self._link_last:
            return
        self._link_last = key
        self.lbl_link.setText(msg)
        self.lbl_link.setStyleSheet(
            f"color:{NEON if self._link_ok else AMBER}; font-family:{MONO};"
            "font-size:15px; letter-spacing:2px;")

    # -----------------------------------------------------------------------
    def keyPressEvent(self, e):
        k = e.key()
        if k in (QtCore.Qt.Key_F11, QtCore.Qt.Key_F):
            self.showNormal() if self.isFullScreen() else self.showFullScreen()
        elif k == QtCore.Qt.Key_Escape and self.isFullScreen():
            self.showNormal()
        elif k == QtCore.Qt.Key_Q:
            self.close()

    def closeEvent(self, e):
        self.timer.stop()
        self.reader.stop()
        super().closeEvent(e)


# ===========================================================================
#  6.  ENTRY POINT
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description="Monitor pasien ECG + SpO2 (ESP32)")
    ap.add_argument("--port", default=None, help="COM5 | /dev/ttyUSB0 (default: auto)")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--fs-ecg", type=float, default=FS_ECG, dest="fs_ecg",
                    help="sample rate EKG (Hz), samakan dengan firmware")
    ap.add_argument("--fs-ppg", type=float, default=FS_PPG, dest="fs_ppg",
                    help="sample rate PPG (Hz), samakan dengan firmware")
    ap.add_argument("--mains", type=float, default=50.0, choices=(50.0, 60.0),
                    help="frekuensi listrik untuk notch: 50 (Indonesia/Eropa) atau 60")
    ap.add_argument("--ir-threshold", type=float, default=IR_FINGER_THRESH,
                    dest="ir_threshold", help="IR mentah di bawah nilai ini = tidak ada jari")
    ap.add_argument("--invert", action="store_true",
                    help="balik polaritas pleth jika gelombang PPG tampil terbalik")
    ap.add_argument("--simulate", action="store_true", help="data sintetis, tanpa hardware")
    g = ap.add_argument_group("MQTT (alat jauh via WiFi + broker)")
    g.add_argument("--mqtt", default=None, metavar="HOST",
                   help="alamat broker MQTT; bila diisi, data diambil dari broker (bukan USB)")
    g.add_argument("--mqtt-port", type=int, default=None, dest="mqtt_port",
                   help="port broker (default 1883, atau 8883 bila --tls)")
    g.add_argument("--device-id", default="esp32-01", dest="device_id",
                   help="id alat, harus sama dengan DEVICE_ID di firmware")
    g.add_argument("--topic-prefix", default=MQTT_PREFIX, dest="topic_prefix")
    g.add_argument("--user", default=None, help="user broker (opsional)")
    g.add_argument("--password", default=os.environ.get("MQTT_PASSWORD"),
                   help="password broker (atau set variabel lingkungan MQTT_PASSWORD)")
    g.add_argument("--tls", action="store_true", help="pakai TLS (disarankan lewat internet)")
    g.add_argument("--ca-file", default=None, dest="ca_file",
                   help="berkas CA untuk broker dengan sertifikat sendiri")
    g.add_argument("--tls-insecure", action="store_true", dest="tls_insecure",
                   help="TLS tanpa verifikasi nama host (HANYA uji coba)")
    g.add_argument("--jitter-ms", type=int, default=JITTER_MS, dest="jitter_ms",
                   help="tundaan jitter buffer (ms); naikkan bila gelombang tersendat")
    ap.add_argument("--windowed", action="store_true", help="jangan mulai fullscreen")
    args = ap.parse_args()
    if args.mqtt_port is None:
        args.mqtt_port = 8883 if args.tls else 1883
    if args.mqtt and args.simulate:
        ap.error("--mqtt dan --simulate tidak dapat dipakai bersamaan")

    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(f"QWidget{{background:{BG};}}")

    win = MonitorWindow(args)
    win.showMaximized() if args.windowed else win.showFullScreen()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
