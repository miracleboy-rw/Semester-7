#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
 SIMULATOR ALAT (pengganti ESP32) UNTUK MQTT
=============================================================================
 Menerbitkan data EKG + PPG sintetis ke broker MQTT dengan format PERSIS sama
 seperti firmware ESP32, sehingga broker dan aplikasi monitor dapat diuji
 tanpa hardware. Berdiri sendiri (tidak butuh PyQt5), cukup:  pip install paho-mqtt numpy

 Contoh:
     python mqtt_device_sim.py --host 192.168.1.10
     python mqtt_device_sim.py --host broker.contoh.com --tls --user alat --password rahasia
     python mqtt_device_sim.py --host localhost --faults       # + elektrode/jari lepas berkala
     python mqtt_device_sim.py --dry-run                       # tanpa broker, cek generator saja

 lalu di komputer monitor:
     python ecg_spo2_monitor.py --mqtt 192.168.1.10 --device-id esp32-01

 Format paket stream (little-endian, header 20 byte) -- lihat komentar di firmware:
   ver u8 | flags u8 | seq u32 | ecgIdx u32 | nE u8 | ppgIdx u32 | nP u8 |
   ppgBpm u8 | spo2 u8 | ecgBpm u16 | nE x u16 | nP x (u16 IR, u16 RED)
=============================================================================
"""
import argparse
import json
import math
import os
import struct
import sys
import time

import numpy as np

FS_ECG, FS_PPG = 200, 100
BATCH_E, BATCH_P = 20, 10                       # per 100 ms
HDR = struct.Struct("<BBIIBIBBBH")


def encode_stream(seq, ecg_idx, ecg, ppg_idx, ppg, flags, ppg_bpm, spo2, ecg_bpm):
    """ppg: array (nP, 2) [IR, RED]. Mengembalikan bytes."""
    head = HDR.pack(1, flags, seq & 0xFFFFFFFF, ecg_idx & 0xFFFFFFFF, len(ecg),
                    ppg_idx & 0xFFFFFFFF, len(ppg), ppg_bpm, spo2, ecg_bpm)
    return head + np.asarray(ecg, "<u2").tobytes() + np.asarray(ppg, "<u2").tobytes()


class Generator:
    """EKG (PQRST) 200 Hz dan PPG 100 Hz sinkron; HR dan SpO2 dapat diatur."""

    def __init__(self, hr=75.0, faults=False, seed=0):
        self.hr, self.faults = hr, faults
        self.rng = np.random.default_rng(seed)
        self.phase = 0.0
        self.k = 0                               # nomor sampel EKG

    def state(self, t):
        leads_off = self.faults and (t % 60.0) >= 54.0
        finger_off = self.faults and 38.0 <= (t % 45.0)
        return leads_off, finger_off

    def batch(self):
        """Hasilkan satu batch 100 ms: (ecg[20], ppg[10,2], leads_off, finger_off)."""
        ecg, ppg = [], []
        t0 = self.k / FS_ECG
        leads_off, finger_off = self.state(t0)
        for _ in range(BATCH_E):
            t = self.k / FS_ECG
            hr = self.hr + 4.0 * math.sin(2 * math.pi * 0.05 * t)
            self.phase = (self.phase + hr / 60.0 / FS_ECG) % 1.0
            ph = self.phase
            v = (0.10 * math.exp(-((ph - 0.20) ** 2) / 0.0009)
                 - 0.15 * math.exp(-((ph - 0.47) ** 2) / 0.00015)
                 + 1.00 * math.exp(-((ph - 0.50) ** 2) / 0.00012)
                 - 0.25 * math.exp(-((ph - 0.53) ** 2) / 0.00020)
                 + 0.30 * math.exp(-((ph - 0.70) ** 2) / 0.0025))
            v += 0.05 * math.sin(2 * math.pi * 50 * t) + 0.02 * self.rng.standard_normal()
            raw = int(max(0, min(4095, 2048 + v * 900)))
            ecg.append(2048 if leads_off else raw)
            if self.k % 2 == 0:                  # PPG 100 Hz
                pp = (ph - 0.5) % 1.0
                w = math.exp(-((pp - 0.18) ** 2) / 0.006) + 0.33 * math.exp(-((pp - 0.42) ** 2) / 0.012)
                if finger_off:
                    ppg.append((int(900 + self.rng.normal(0, 60)), int(800 + self.rng.normal(0, 60))))
                else:
                    ppg.append((int(48000 - 2600 * w + self.rng.normal(0, 90)),
                                int(39000 - 1050 * w + self.rng.normal(0, 90))))
            self.k += 1
        return np.array(ecg), np.array(ppg), leads_off, finger_off


def make_client(args):
    import paho.mqtt.client as mqtt
    cid = f"{args.device_id}-sim-{os.getpid() % 10000}"
    try:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid)    # paho >= 2.0
    except AttributeError:
        c = mqtt.Client(client_id=cid)                                       # paho 1.x
    if args.user:
        c.username_pw_set(args.user, args.password)
    if args.tls:
        c.tls_set(ca_certs=args.ca_file or None)
        if args.tls_insecure:
            c.tls_insecure_set(True)
    return c


def main():
    ap = argparse.ArgumentParser(description="Simulator alat ESP32 untuk MQTT")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--device-id", default="esp32-01", dest="device_id")
    ap.add_argument("--topic-prefix", default="monitor", dest="prefix")
    ap.add_argument("--user", default=None)
    ap.add_argument("--password", default=os.environ.get("MQTT_PASSWORD"))
    ap.add_argument("--tls", action="store_true")
    ap.add_argument("--ca-file", default=None, dest="ca_file")
    ap.add_argument("--tls-insecure", action="store_true", dest="tls_insecure")
    ap.add_argument("--hr", type=float, default=75.0, help="HR rata-rata (bpm)")
    ap.add_argument("--spo2", type=int, default=97)
    ap.add_argument("--faults", action="store_true", help="simulasikan elektrode/jari lepas berkala")
    ap.add_argument("--dry-run", action="store_true", dest="dry", help="tanpa broker; hanya cek generator")
    a = ap.parse_args()
    if a.port is None:
        a.port = 8883 if a.tls else 1883

    gen = Generator(a.hr, a.faults)
    base = f"{a.prefix}/{a.device_id}"
    t_stream, t_vitals, t_status, t_alarm = (base + "/stream", base + "/vitals",
                                             base + "/status", base + "/alarm")
    seq = 0
    pidx = 0

    client = None
    if not a.dry:
        client = make_client(a)
        client.will_set(t_status, "offline", qos=1, retain=True)       # Last Will, seperti firmware
        client.reconnect_delay_set(min_delay=1, max_delay=10)
        client.connect(a.host, a.port, keepalive=15)
        client.loop_start()
        time.sleep(0.5)
        client.publish(t_status, "online", qos=1, retain=True)
        print(f"Terhubung ke {a.host}:{a.port}, menerbitkan ke {base}/...  (Ctrl-C untuk berhenti)")

    t0 = time.perf_counter()
    next_t = t0
    sent = 0
    last_vitals = 0.0
    last_alarm_mask = None
    try:
        while True:
            now = time.perf_counter()
            if now < next_t:
                time.sleep(min(0.01, next_t - now))
                continue
            next_t += BATCH_E / FS_ECG                   # 100 ms
            ecg_idx = gen.k
            ecg, ppg, loff, foff = gen.batch()
            flags = (1 if loff else 0) | (2 if foff else 0)
            ppg_bpm = 0 if foff else int(round(a.hr))
            spo2 = 0 if foff else a.spo2
            payload = encode_stream(seq, ecg_idx, ecg, pidx, ppg, flags, ppg_bpm, spo2,
                                    0 if loff else int(round(a.hr)))
            seq += 1
            pidx += len(ppg)
            sent += 1
            if client:
                client.publish(t_stream, payload, qos=0)
                el = now - t0
                if el - last_vitals >= 1.0:
                    last_vitals = el
                    client.publish(t_vitals, json.dumps({
                        "hr": 0 if loff else int(round(a.hr)), "pr": ppg_bpm, "spo2": spo2,
                        "loff": int(loff), "foff": int(foff), "danger": 0, "mask": 0,
                        "rssi": -55, "up": int(el), "sent": sent, "drop": 0}))
                mask = 0
                if mask != last_alarm_mask:
                    last_alarm_mask = mask
                    client.publish(t_alarm, json.dumps({"alarm": 0, "mask": 0}), qos=1, retain=True)
            elif sent >= 100:                            # dry-run: 10 detik data
                print(f"dry-run OK: {sent} paket, {len(payload)} byte/paket "
                      f"({len(payload) * 10 / 1000:.2f} KB/detik) HR {a.hr}")
                return
    except KeyboardInterrupt:
        pass
    finally:
        if client:
            client.publish(t_status, "offline", qos=1, retain=True).wait_for_publish(2)
            client.loop_stop()
            client.disconnect()
        print("berhenti.")


if __name__ == "__main__":
    main()
