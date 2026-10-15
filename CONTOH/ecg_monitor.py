#!/usr/bin/env python3
"""
Clinical-grade ECG patient-monitor replica for the AD8232 + Arduino rig.

Architecture
------------
    [Serial USB]  ->  Ingestion thread  ->  thread-safe FIFO Queue
                                               |
                                     Main GUI thread (60 FPS QTimer)
                                               |
                     DSP pipeline: High-pass -> Low-pass -> 50 Hz Notch
                                               |
                     Pan-Tompkins-lite R-peak detection -> live BPM
                                               |
                     PyQtGraph sweep render (black canvas, neon trace,
                     erasing wiper bar, 70/30 clinical dashboard)

Serial protocol (from ecg_monitor.ino, 115200 baud, ~200 Hz)
    raw,leadsOff,beat,bpm    e.g.  "531,0,1,72"

Usage
    pip install pyqtgraph PyQt5 pyserial scipy numpy
    python ecg_monitor.py --port COM3            # Windows
    python ecg_monitor.py --port /dev/ttyUSB0    # Linux / macOS
    python ecg_monitor.py --simulate             # no hardware, synthetic ECG
"""

import sys
import time
import queue
import argparse
import threading
from collections import deque

import numpy as np
from scipy.signal import butter, iirnotch, lfilter, lfilter_zi

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui, QtWidgets

# ----------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------
FS            = 200          # sampling rate (must match the Arduino sketch)
WINDOW_SEC    = 5            # seconds of ECG visible on screen
N_WIN         = FS * WINDOW_SEC
WIPER_SAMPLES = int(0.05 * N_WIN)   # erasing bar ~5% of screen width
POWER_LINE_HZ = 50.0         # notch target (50 Hz mains); use 60.0 in the Americas

# Heart-rate danger thresholds (must mirror the Arduino zones)
BPM_BRADY = 50
BPM_TACHY = 120

# Clinical palette
COL_BG      = '#000000'
COL_GRID    = '#143c14'
COL_TRACE   = '#00FF00'
COL_BPM     = '#00FF66'
COL_WARN    = '#FFB000'
COL_DIAG    = '#00E5FF'
COL_DANGER  = '#FF3030'


# ======================================================================
# 1. INGESTION WORKER  (background thread)
# ======================================================================
class SerialReader(threading.Thread):
    """Pulls raw bytes off the USB buffer and drops parsed samples into a FIFO."""

    def __init__(self, port, baud, out_queue):
        super().__init__(daemon=True)
        self.port, self.baud = port, baud
        self.q = out_queue
        self.running = True
        self.received = 0
        self.dropped = 0

    def run(self):
        import serial  # imported here so --simulate needs no pyserial
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as e:
            print(f"[SerialReader] cannot open {self.port}: {e}")
            return
        ser.reset_input_buffer()
        while self.running:
            try:
                line = ser.readline().decode('ascii', 'ignore').strip()
            except Exception:
                continue
            if not line:
                continue
            parts = line.split(',')
            if len(parts) != 4:
                self.dropped += 1
                continue
            try:
                raw   = int(parts[0])
                loff  = int(parts[1])
                beat  = int(parts[2])
                bpm   = int(parts[3])
            except ValueError:
                self.dropped += 1
                continue
            self.received += 1
            self.q.put((raw, loff, beat, bpm))
        ser.close()

    def stop(self):
        self.running = False


class Simulator(threading.Thread):
    """Synthetic ECG generator so the UI can be exercised without hardware."""

    def __init__(self, out_queue, bpm=72):
        super().__init__(daemon=True)
        self.q = out_queue
        self.running = True
        self.bpm = bpm
        self.received = 0
        self.dropped = 0

    def run(self):
        t = 0.0
        dt = 1.0 / FS
        rng = np.random.default_rng(0)
        while self.running:
            rr = 60.0 / self.bpm
            phase = (t % rr) / rr
            # crude but recognisable PQRST built from gaussians
            val = (
                 0.10 * np.exp(-((phase - 0.20) ** 2) / 0.0009)   # P
                -0.15 * np.exp(-((phase - 0.47) ** 2) / 0.00015)  # Q
                +1.00 * np.exp(-((phase - 0.50) ** 2) / 0.00012)  # R
                -0.25 * np.exp(-((phase - 0.53) ** 2) / 0.00020)  # S
                +0.30 * np.exp(-((phase - 0.70) ** 2) / 0.0025)   # T
            )
            val += 0.05 * np.sin(2 * np.pi * POWER_LINE_HZ * t)   # mains hum
            val += 0.02 * rng.standard_normal()                   # muscle noise
            val += 0.10 * np.sin(2 * np.pi * 0.25 * t)            # baseline wander
            raw = int(512 + val * 300)
            raw = max(0, min(1023, raw))
            self.received += 1
            self.q.put((raw, 0, 0, int(self.bpm)))
            t += dt
            time.sleep(dt)

    def stop(self):
        self.running = False


# ======================================================================
# 2. DSP PIPELINE  (stateful, real-time IIR filters)
# ======================================================================
class ECGFilter:
    """Sequential High-pass -> Low-pass -> Notch, sample-accurate via zi state."""

    def __init__(self, fs):
        nyq = fs / 2.0
        # Stage 1: High-pass 0.5 Hz  -> kills baseline wander
        self.hp_b, self.hp_a = butter(2, 0.5 / nyq, btype='highpass')
        # Stage 2: Low-pass 40 Hz    -> smooths muscle tremor
        self.lp_b, self.lp_a = butter(4, 40.0 / nyq, btype='lowpass')
        # Stage 3: Notch at mains    -> removes AC grid hum
        self.no_b, self.no_a = iirnotch(POWER_LINE_HZ / nyq, Q=30.0)
        # per-stage running state
        self.hp_zi = lfilter_zi(self.hp_b, self.hp_a) * 0.0
        self.lp_zi = lfilter_zi(self.lp_b, self.lp_a) * 0.0
        self.no_zi = lfilter_zi(self.no_b, self.no_a) * 0.0

    def process(self, x):
        """Filter a 1-D numpy block, carrying state across calls."""
        if len(x) == 0:
            return x
        y, self.hp_zi = lfilter(self.hp_b, self.hp_a, x, zi=self.hp_zi)
        y, self.lp_zi = lfilter(self.lp_b, self.lp_a, y, zi=self.lp_zi)
        y, self.no_zi = lfilter(self.no_b, self.no_a, y, zi=self.no_zi)
        return y


# ======================================================================
# 3. LIVE MATH ENGINE  (Pan-Tompkins-lite R-peak / BPM)
# ======================================================================
class BeatDetector:
    """Self-calibrating peak detector: threshold rides on the signal's own noise.

    Tracks a running mean (baseline) and standard deviation of the filtered
    signal with a slow exponential average. An R-peak is any sample that
    punches K standard deviations above baseline. This adapts automatically
    to small OR large signals, so weak electrode leads still get counted.
    """

    def __init__(self, fs):
        self.fs = fs
        self.mu = 0.0                         # running baseline
        self.var = 0.02                       # running variance (noise power)
        self.alpha = 1.0 / (0.6 * fs)         # ~0.6 s adaptation time constant
        self.K = 3.5                          # peaks must exceed mu + K*std
        self.last_peak_t = None
        self.refractory = 0.28                # 280 ms lockout (=> <=210 bpm)
        self.armed = True
        self.bpm = 0

    def update(self, sample, now):
        """Feed one filtered sample; return True on a detected R-peak."""
        # slow update of baseline + noise estimate (R-waves are too brief to skew it)
        d = sample - self.mu
        self.mu += self.alpha * d
        self.var += self.alpha * (d * d - self.var)
        std = max(self.var ** 0.5, 1e-4)
        thr = self.mu + self.K * std          # dynamic ceiling, in noise units

        beat = False
        if self.armed and sample > thr:
            if self.last_peak_t is None or (now - self.last_peak_t) > self.refractory:
                if self.last_peak_t is not None:
                    rr = now - self.last_peak_t
                    if 0.25 < rr < 3.0:       # 20..240 bpm sanity window
                        inst = 60.0 / rr
                        # light smoothing so the readout is steady
                        self.bpm = int(round(inst if self.bpm == 0
                                             else 0.6 * self.bpm + 0.4 * inst))
                self.last_peak_t = now
                self.armed = False
                beat = True
        # re-arm once the signal drops back toward baseline
        if sample < self.mu + 0.5 * std:
            self.armed = True
        return beat


# ======================================================================
# 4. GRAPHIC PROCESSING ENGINE  (main GUI thread, 60 FPS)
# ======================================================================
class MonitorWindow(QtWidgets.QWidget):
    def __init__(self, reader, in_queue):
        super().__init__()
        self.reader = reader
        self.q = in_queue
        self.filt = ECGFilter(FS)
        self.det = BeatDetector(FS)

        # ring buffer for the sweep
        self.buf = np.full(N_WIN, np.nan)
        self.x = np.arange(N_WIN) / FS
        self.write_idx = 0

        self.leads_off = False
        self.hw_bpm = 0
        self.total = 0
        self.beat_icon_until = 0.0
        self.t0 = time.perf_counter()

        # auto-gain: scale a weak signal up so it fills the screen
        self.env_peak = 0.5      # decaying estimate of recent |signal| peak
        self.disp_gain = 1.0

        self._build_ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(16)   # ~60 FPS

    # -------------------------------------------------- UI
    def _build_ui(self):
        self.setWindowTitle('ECG Patient Monitor')
        self.setStyleSheet(f'background-color:{COL_BG};')
        self.resize(1200, 640)

        root = QtWidgets.QHBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(8)

        # ---- Left 70%: waveform ----
        pg.setConfigOptions(antialias=True, background=COL_BG, foreground='#888888')
        self.plot = pg.PlotWidget()
        self.plot.setYRange(-1.2, 1.6, padding=0)
        self.plot.setXRange(0, WINDOW_SEC, padding=0)
        self.plot.hideButtons()
        self.plot.setMenuEnabled(False)
        self.plot.setMouseEnabled(x=False, y=False)
        self.plot.getPlotItem().hideAxis('left')
        self.plot.getPlotItem().hideAxis('bottom')
        # ECG-paper grid: major 1 s + minor 0.2 s
        self._draw_grid()
        self.curve = self.plot.plot(self.x, self.buf,
                                    pen=pg.mkPen(COL_TRACE, width=2))
        self.wiper = pg.InfiniteLine(pos=0, angle=90,
                                     pen=pg.mkPen('#003300', width=int(0.05 * 400)))
        self.plot.addItem(self.wiper)
        root.addWidget(self.plot, 70)

        # ---- Right 30%: dashboard ----
        panel = QtWidgets.QVBoxLayout()
        panel.setSpacing(10)

        # BPM readout (top)
        self.bpm_title = self._label('HR  bpm', COL_BPM, 18)
        self.bpm_value = self._label('--', COL_BPM, 96, bold=True)
        self.bpm_value.setAlignment(QtCore.Qt.AlignCenter)
        panel.addWidget(self.bpm_title)
        panel.addWidget(self.bpm_value)

        # Leads-off warning (middle) - hidden until triggered
        self.leads_box = self._label('LEADS OFF\nCHECK ELECTRODES', COL_BG, 18, bold=True)
        self.leads_box.setAlignment(QtCore.Qt.AlignCenter)
        self.leads_box.setStyleSheet(
            f'background-color:{COL_BG}; color:{COL_BG}; padding:14px;')
        panel.addWidget(self.leads_box)

        panel.addStretch(1)

        # System diagnostics (bottom)
        self.diag = self._label('Sampling: -- Hz | Drop: --%', COL_DIAG, 12)
        panel.addWidget(self.diag)

        wrap = QtWidgets.QWidget()
        wrap.setLayout(panel)
        wrap.setFixedWidth(340)
        root.addWidget(wrap, 30)

    def _label(self, text, color, size, bold=False):
        lab = QtWidgets.QLabel(text)
        w = 'bold' if bold else 'normal'
        lab.setStyleSheet(
            f'color:{color}; font-family:Consolas,monospace; '
            f'font-size:{size}px; font-weight:{w};')
        return lab

    def _draw_grid(self):
        pi = self.plot.getPlotItem()
        minor = pg.mkPen(COL_GRID, width=1)
        major = pg.mkPen('#1f5c1f', width=1)
        # vertical lines every 0.2 s (minor) and 1 s (major)
        n = 0
        t = 0.0
        while t <= WINDOW_SEC + 1e-6:
            pen = major if abs(t - round(t)) < 1e-6 else minor
            pi.addItem(pg.InfiniteLine(pos=t, angle=90, pen=pen))
            t += 0.2
            n += 1
        # horizontal lines every 0.2 mV-equivalent
        y = -1.2
        while y <= 1.6 + 1e-6:
            pen = major if abs((y * 5) - round(y * 5)) < 1e-6 and int(round(y*5)) % 5 == 0 else minor
            pi.addItem(pg.InfiniteLine(pos=y, angle=0, pen=minor))
            y += 0.2

    # -------------------------------------------------- 60 FPS loop
    def _tick(self):
        # 1) drain the FIFO
        raws, loff_last, hwbpm_last = [], self.leads_off, self.hw_bpm
        drained = 0
        while True:
            try:
                raw, loff, beat, bpm = self.q.get_nowait()
            except queue.Empty:
                break
            raws.append(raw)
            loff_last = bool(loff)
            hwbpm_last = bpm
            drained += 1
            if drained > 4 * N_WIN:      # safety valve
                break

        self.leads_off = loff_last
        self.hw_bpm = hwbpm_last
        self.total += drained

        if raws:
            # 2) DSP: normalise to ~[-1,1] then run the 3-stage filter
            x = (np.asarray(raws, dtype=float) - 512.0) / 300.0
            y = self.filt.process(x)

            # 3) peak detection + write into sweep buffer
            now = time.perf_counter()
            for s in y:
                if self.det.update(s, now):
                    self.beat_icon_until = now + 0.12
                # auto-gain: track a slowly-decaying peak and scale to fill screen
                a = abs(s)
                if a > self.env_peak:
                    self.env_peak = a
                else:
                    self.env_peak *= 0.9995          # ~3 s decay
                self.disp_gain = min(0.9 / max(self.env_peak, 0.05), 8.0)
                v = s * self.disp_gain               # displayed amplitude
                self.buf[self.write_idx] = max(-1.15, min(1.55, v))
                # erasing wiper: blank the samples just ahead of the cursor
                for k in range(1, WIPER_SAMPLES):
                    self.buf[(self.write_idx + k) % N_WIN] = np.nan
                self.write_idx = (self.write_idx + 1) % N_WIN

        self._render()

    def _render(self):
        self.curve.setData(self.x, self.buf, connect='finite')
        self.wiper.setValue(self.write_idx / FS)

        # BPM source: prefer Python detection, fall back to Arduino value
        bpm = self.det.bpm or self.hw_bpm
        danger = (not self.leads_off) and bpm > 0 and (bpm > BPM_TACHY or bpm < BPM_BRADY)

        if self.leads_off:
            self.bpm_value.setText('--')
        else:
            beating = time.perf_counter() < self.beat_icon_until
            heart = ' ♥' if beating else ''
            self.bpm_value.setText(f'{bpm}{heart}')

        col = COL_DANGER if danger else COL_BPM
        self.bpm_value.setStyleSheet(
            f'color:{col}; font-family:Consolas,monospace; font-size:96px; font-weight:bold;')

        # leads-off warning box (blinks)
        if self.leads_off and int(time.perf_counter() * 2) % 2 == 0:
            self.leads_box.setStyleSheet(
                f'background-color:{COL_WARN}; color:#000000; padding:14px; '
                f'font-family:Consolas,monospace; font-size:18px; font-weight:bold;')
        else:
            self.leads_box.setStyleSheet(
                f'background-color:{COL_BG}; color:{COL_BG}; padding:14px; '
                f'font-family:Consolas,monospace; font-size:18px; font-weight:bold;')

        # diagnostics
        recv = getattr(self.reader, 'received', 0)
        drop = getattr(self.reader, 'dropped', 0)
        rate = recv + drop
        drop_pct = (100.0 * drop / rate) if rate else 0.0
        elapsed = max(time.perf_counter() - self.t0, 1e-3)
        eff_hz = recv / elapsed
        self.diag.setText(f'Sampling: {eff_hz:5.1f} Hz | Drop Rate: {drop_pct:.1f}%')

    def closeEvent(self, ev):
        self.reader.stop()
        ev.accept()


# ======================================================================
# main
# ======================================================================
def main():
    ap = argparse.ArgumentParser(description='ECG patient-monitor replica')
    ap.add_argument('--port', help='serial port, e.g. COM3 or /dev/ttyUSB0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--simulate', action='store_true',
                    help='run with a synthetic ECG (no hardware needed)')
    args = ap.parse_args()

    q = queue.Queue()
    if args.simulate or not args.port:
        if not args.simulate:
            print('No --port given; starting in --simulate mode.')
        reader = Simulator(q)
    else:
        reader = SerialReader(args.port, args.baud, q)
    reader.start()

    app = QtWidgets.QApplication(sys.argv)
    win = MonitorWindow(reader, q)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()