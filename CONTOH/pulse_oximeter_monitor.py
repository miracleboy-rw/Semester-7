#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
=============================================================================
 CLINICAL PPG MONITOR  --  MAX30100 / Arduino  ->  PyQt5 + pyqtgraph
=============================================================================
 A patient-side-monitor style visualiser for a MAX30100 pulse oximeter.

   * Pitch black canvas, neon-cyan phosphor palette, dim medical grid
   * 70 / 30 split: sweeping PPG waveform | giant SpO2 + BPM readouts
   * Non-scrolling "erase bar" sweep with a bright head and fading trail
   * Streaming Butterworth band-pass (0.5 - 5 Hz) via scipy.signal.sosfilt
   * Dedicated QThread for serial I/O -> thread-safe deque -> 60 FPS GUI
   * Finger-off detection, flashing probe alarm, auto serial reconnect

 Expected serial line format (115200 baud, ASCII, newline terminated):

       <rawIR>,<rawRED>,<bpm>,<spo2>\n
       e.g.  48213,39120,72.4,97

 Lines starting with '#' are treated as comments. Missing bpm/spo2 fields
 are tolerated (they will simply read '--').

 Usage
 -----
   python clinical_monitor.py                 # auto-detect the Arduino
   python clinical_monitor.py --port COM5
   python clinical_monitor.py --port /dev/ttyUSB0 --fs 100
   python clinical_monitor.py --simulate      # no hardware needed (demo)

 Keys:  F11 / F  toggle fullscreen      Esc  exit fullscreen      Q  quit
=============================================================================
"""

import sys
import time
import math
import argparse
import collections

import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi

from PyQt5 import QtCore, QtGui, QtWidgets
import pyqtgraph as pg

try:
    import serial
    from serial.tools import list_ports
except ImportError:                                    # allow --simulate w/o pyserial
    serial = None
    list_ports = None


# ---------------------------------------------------------------------------
# CONFIGURATION
# ---------------------------------------------------------------------------
BAUD              = 115200
FS                = 100.0          # sensor sample rate (Hz) - must match sketch
WINDOW_SECONDS    = 5.0            # width of one sweep across the screen
BLANK_FRACTION    = 0.05           # erase bar width, 5 % of the sweep
TRAIL_SAMPLES     = 55             # length of the bright trail behind the head
GUI_FPS           = 60

BP_LOW, BP_HIGH   = 0.5, 5.0       # band-pass corners (Hz)
BP_ORDER          = 2              # per band -> 4th order overall

IR_FINGER_THRESH  = 5000           # raw IR below this  ==  no finger on probe
SPO2_ALARM_LOW    = 90             # %
BPM_ALARM_LOW     = 45
BPM_ALARM_HIGH    = 130

# --- palette ---------------------------------------------------------------
BG          = "#000000"
NEON        = "#00FFFF"
NEON_DIM    = "#00A0A8"
GRID_MINOR  = (0, 255, 255, 26)
GRID_MAJOR  = (0, 255, 255, 54)
ALARM       = "#FF2A2A"
AMBER       = "#FFB000"

MONO = "Consolas, 'DejaVu Sans Mono', 'Courier New', monospace"


# ===========================================================================
#  1.  SERIAL ACQUISITION THREAD
# ===========================================================================
class SerialReader(QtCore.QThread):
    """
    Runs entirely off the GUI thread. Its only jobs are:
        open/reopen the port -> read a line -> parse -> append to the deque.
    No filtering, no drawing, no Qt widget access. Ever.
    """
    status = QtCore.pyqtSignal(str, bool)      # (message, is_connected)

    def __init__(self, queue, port=None, baud=BAUD, simulate=False, parent=None):
        super().__init__(parent)
        self.queue    = queue
        self.port     = port
        self.baud     = baud
        self.simulate = simulate
        self._running = True
        self._ser     = None

    # -- public ------------------------------------------------------------
    def stop(self):
        self._running = False
        self.wait(1500)
        self._close()

    # -- internals ---------------------------------------------------------
    def _close(self):
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        self._ser = None

    @staticmethod
    def _autodetect():
        """Pick the first port that smells like an Arduino / USB-serial bridge."""
        if list_ports is None:
            return None
        candidates = list(list_ports.comports())
        keys = ("arduino", "ch340", "ch910", "cp210", "ftdi", "wch", "usb serial",
                "usbmodem", "usbserial", "acm")
        for p in candidates:
            blob = f"{p.description} {p.manufacturer} {p.device}".lower()
            if any(k in blob for k in keys):
                return p.device
        return candidates[0].device if candidates else None

    @staticmethod
    def _parse(line):
        """'ir,red,bpm,spo2' -> (ir, red, bpm, spo2) or None on garbage."""
        if not line or line[0] in "#<":
            return None
        parts = line.replace(";", ",").replace("\t", ",").split(",")
        if len(parts) < 2:
            return None
        try:
            ir  = float(parts[0])
            red = float(parts[1])
        except ValueError:
            return None                       # corrupted packet -> silently drop
        def opt(i):
            try:
                return float(parts[i])
            except (IndexError, ValueError):
                return 0.0
        return (ir, red, opt(2), opt(3))

    # -- thread body -------------------------------------------------------
    def run(self):
        if self.simulate:
            self._run_simulation()
            return

        if serial is None:
            self.status.emit("pyserial NOT INSTALLED", False)
            return

        backoff = 0.0
        while self._running:
            # ---------- (re)connect ----------
            if self._ser is None:
                try:
                    port = self.port or self._autodetect()
                    if port is None:
                        self.status.emit("NO SERIAL PORT FOUND - SEARCHING", False)
                        time.sleep(1.0)
                        continue
                    self.status.emit(f"OPENING {port} @ {self.baud}", False)
                    self._ser = serial.Serial(port, self.baud, timeout=1.0)
                    time.sleep(2.0)                    # ride out the auto-reset
                    self._ser.reset_input_buffer()
                    self.status.emit(f"LINK OK  {port} @ {self.baud} BAUD", True)
                    backoff = 0.0
                except Exception as exc:
                    self._close()
                    self.status.emit(f"PORT ERROR: {type(exc).__name__} - RETRYING", False)
                    backoff = min(backoff + 0.5, 3.0)
                    time.sleep(backoff)
                    continue

            # ---------- read ----------
            try:
                raw = self._ser.readline()
                if not raw:
                    continue                            # timeout, keep the link
                sample = self._parse(raw.decode("ascii", errors="ignore").strip())
                if sample is not None:
                    self.queue.append(sample)
            except (OSError, serial.SerialException) as exc:
                # USB yanked mid-stream: drop the handle and re-enter the
                # reconnect branch on the next loop instead of dying.
                self._close()
                self.status.emit(f"LINK LOST ({type(exc).__name__}) - RECONNECTING", False)
                time.sleep(1.0)
            except Exception:
                continue                                # never let one bad byte kill us

        self._close()

    # -- offline demo generator -------------------------------------------
    def _run_simulation(self):
        self.status.emit("SIMULATION MODE - NO HARDWARE", True)
        t, dt = 0.0, 1.0 / FS
        nxt = time.perf_counter()
        while self._running:
            hr    = 72 + 6 * math.sin(2 * math.pi * 0.05 * t)      # gentle HRV
            phase = (t * hr / 60.0) % 1.0
            # crude but convincing PPG: systolic peak + dicrotic notch
            wave = (math.exp(-((phase - 0.18) ** 2) / 0.006) * 1.0 +
                    math.exp(-((phase - 0.42) ** 2) / 0.012) * 0.33)
            resp    = 0.35 * math.sin(2 * math.pi * 0.25 * t)      # baseline wander
            finger  = (t % 45.0) < 38.0                            # unplug demo
            if finger:
                ir  = 48000 + 2600 * wave + 1800 * resp + np.random.normal(0, 90)
                red = 39000 + 1750 * wave + 1400 * resp + np.random.normal(0, 90)
                bpm, spo2 = hr, 97 + 1.2 * math.sin(2 * math.pi * 0.03 * t)
            else:
                ir  = 900 + np.random.normal(0, 60)
                red = 800 + np.random.normal(0, 60)
                bpm, spo2 = 0, 0
            self.queue.append((ir, red, bpm, spo2))
            t += dt
            nxt += dt
            time.sleep(max(0.0, nxt - time.perf_counter()))


# ===========================================================================
#  2.  STREAMING DSP
# ===========================================================================
class StreamingBandpass:
    """
    Stateful 2nd-order-sections Butterworth band-pass.

    Block-wise filtering with a persisted `zi` means chunk boundaries are
    seamless - the output is bit-identical to filtering the whole record at
    once, which is what stops the waveform from twitching every frame.
    """
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


class BeatFlagger:
    """Threshold + refractory peak marker, purely for the blinking heart icon."""
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
#  3.  UI BUILDING BLOCKS
# ===========================================================================
class VitalCard(QtWidgets.QFrame):
    """One giant numeric readout: label, value, unit, plus an alarm state."""

    def __init__(self, title, unit, color=NEON, value_pt=150, parent=None):
        super().__init__(parent)
        self.base_color = color
        self.setStyleSheet(f"background:{BG}; border:none;")

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(18, 4, 18, 4)
        lay.setSpacing(0)

        top = QtWidgets.QHBoxLayout()
        self.title = QtWidgets.QLabel(title)
        self.title.setStyleSheet(
            f"color:{color}; font-family:{MONO}; font-size:26px;"
            "font-weight:bold; letter-spacing:3px;")
        self.unit = QtWidgets.QLabel(unit)
        self.unit.setStyleSheet(
            f"color:{NEON_DIM}; font-family:{MONO}; font-size:20px;")
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
            f"color:{NEON_DIM}; font-family:{MONO}; font-size:16px;"
            "letter-spacing:2px;")
        lay.addWidget(self.sub)

    def set_value(self, text, alarm=False, dim=False):
        col = ALARM if alarm else (NEON_DIM if dim else self.base_color)
        self.value.setText(text)
        self.value.setStyleSheet(f"color:{col};")


class Separator(QtWidgets.QFrame):
    def __init__(self):
        super().__init__()
        self.setFixedHeight(1)
        self.setStyleSheet(f"background:{NEON_DIM};")


# ===========================================================================
#  4.  MAIN WINDOW
# ===========================================================================
class MonitorWindow(QtWidgets.QMainWindow):

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.fs   = args.fs
        self.N    = int(self.fs * WINDOW_SECONDS)          # samples per sweep
        self.blank = max(4, int(self.N * BLANK_FRACTION))

        # ---- shared state -------------------------------------------------
        self.queue = collections.deque(maxlen=20000)       # deque = thread safe
        self.buf   = np.full(self.N, np.nan)               # display ring buffer
        self.wr    = 0                                     # sweep cursor
        self.x     = np.arange(self.N, dtype=np.float64)

        self.bp        = StreamingBandpass(self.fs, BP_LOW, BP_HIGH)
        self.beat      = BeatFlagger(self.fs)
        self.gain      = 1.0
        self.env_amp   = 0.0
        self.perf      = 0.0
        self.ir_ema    = 0.0
        self.finger    = False
        self.bpm       = 0.0
        self.spo2      = 0.0
        self.last_rx   = 0.0
        self.sps       = 0.0
        self._rx_count = 0
        self._rx_t0    = time.perf_counter()
        self._blink    = True
        self._beat_on  = 0

        self._build_ui()

        # ---- acquisition thread -------------------------------------------
        self.reader = SerialReader(self.queue, args.port, args.baud, args.simulate)
        self.reader.status.connect(self._on_status)
        self.reader.start()

        # ---- timers ---------------------------------------------------------
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

    # -----------------------------------------------------------------
    #  UI CONSTRUCTION
    # -----------------------------------------------------------------
    def _build_ui(self):
        self.setWindowTitle("PPG PATIENT MONITOR  --  MAX30100")
        self.resize(1280, 720)
        self.setMinimumSize(1024, 600)
        self.setStyleSheet(f"background:{BG};")

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # ---------- top status strip ----------
        strip = QtWidgets.QWidget()
        strip.setFixedHeight(34)
        strip.setStyleSheet(f"background:{BG};")
        sl = QtWidgets.QHBoxLayout(strip)
        sl.setContentsMargins(16, 0, 16, 0)

        self.lbl_link = QtWidgets.QLabel("STARTING…")
        self.lbl_link.setStyleSheet(
            f"color:{NEON_DIM}; font-family:{MONO}; font-size:15px; letter-spacing:2px;")
        self.lbl_mode = QtWidgets.QLabel("ADULT   PLETH  II   0.5-5 Hz BPF")
        self.lbl_mode.setStyleSheet(
            f"color:{NEON_DIM}; font-family:{MONO}; font-size:15px; letter-spacing:2px;")
        self.lbl_clock = QtWidgets.QLabel("--:--:--")
        self.lbl_clock.setStyleSheet(
            f"color:{NEON}; font-family:{MONO}; font-size:15px; letter-spacing:2px;")
        sl.addWidget(self.lbl_link)
        sl.addStretch(1)
        sl.addWidget(self.lbl_mode)
        sl.addStretch(1)
        sl.addWidget(self.lbl_clock)
        root.addWidget(strip)

        # ---------- 70 / 30 body ----------
        body = QtWidgets.QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        root.addLayout(body, 1)

        body.addWidget(self._build_plot(), 70)
        body.addWidget(self._build_panel(), 30)

    # -----------------------------------------------------------------
    def _build_plot(self):
        pg.setConfigOptions(antialias=True, background=BG, foreground=NEON)

        self.plot = pg.PlotWidget()
        pi = self.plot.getPlotItem()
        pi.hideButtons()
        pi.setMenuEnabled(False)
        pi.setMouseEnabled(x=False, y=False)
        pi.hideAxis("left")
        pi.hideAxis("bottom")
        pi.setXRange(0, self.N, padding=0)
        pi.setYRange(-1.15, 1.15, padding=0)
        self.plot.setStyleSheet("border:none;")

        # ---- medical graph paper -------------------------------------
        # minor = 0.2 s / 0.2 units, major = 1.0 s / 1.0 unit
        minor_pen = pg.mkPen(GRID_MINOR, width=1)
        major_pen = pg.mkPen(GRID_MAJOR, width=1)
        step = self.N / (WINDOW_SECONDS * 5.0)             # 0.2 s in samples
        for i in range(int(WINDOW_SECONDS * 5) + 1):
            pen = major_pen if i % 5 == 0 else minor_pen
            pi.addItem(pg.InfiniteLine(pos=i * step, angle=90, pen=pen), ignoreBounds=True)
        for j in range(-10, 11):
            pen = major_pen if j % 5 == 0 else minor_pen
            pi.addItem(pg.InfiniteLine(pos=j * 0.2, angle=0, pen=pen), ignoreBounds=True)

        # ---- waveform layers (body -> trail -> head) ------------------
        self.curve_body = pi.plot(pen=pg.mkPen(QtGui.QColor(0, 255, 255, 150), width=1.6))
        self.curve_mid  = pi.plot(pen=pg.mkPen(QtGui.QColor(0, 255, 255, 205), width=2.4))
        self.curve_head = pi.plot(pen=pg.mkPen(QtGui.QColor(190, 255, 255, 255), width=3.2))
        for c in (self.curve_body, self.curve_mid, self.curve_head):
            c.setData(self.x, self.buf, connect="finite")

        self.dot = pg.ScatterPlotItem(size=9, brush=pg.mkBrush(220, 255, 255, 255),
                                      pen=pg.mkPen(0, 255, 255, 255))
        pi.addItem(self.dot)

        # ---- erase / blanking bar -------------------------------------
        self.eraser = pg.LinearRegionItem(values=[0, self.blank], movable=False,
                                          brush=pg.mkBrush(0, 0, 0, 235))
        self.eraser.setZValue(20)
        for ln in self.eraser.lines:
            ln.setPen(pg.mkPen(0, 255, 255, 40))
        pi.addItem(self.eraser)

        # ---- corner caption --------------------------------------------
        self.caption = pg.TextItem("PLETH", color=NEON_DIM, anchor=(0, 0))
        self.caption.setPos(6, 1.10)
        self.caption.setFont(QtGui.QFont("Consolas", 13, QtGui.QFont.Bold))
        pi.addItem(self.caption)

        return self.plot

    # -----------------------------------------------------------------
    def _build_panel(self):
        panel = QtWidgets.QWidget()
        panel.setStyleSheet(f"background:{BG}; border-left:1px solid {NEON_DIM};")
        lay = QtWidgets.QVBoxLayout(panel)
        lay.setContentsMargins(0, 6, 0, 6)
        lay.setSpacing(6)

        self.card_spo2 = VitalCard("SpO2", "%", NEON, value_pt=140)
        self.card_bpm  = VitalCard("PULSE", "bpm", NEON, value_pt=140)
        self.card_spo2.sub.setText(f"LIMIT  >{SPO2_ALARM_LOW}")
        self.card_bpm.sub.setText(f"LIMIT  {BPM_ALARM_LOW}-{BPM_ALARM_HIGH}")

        lay.addWidget(self.card_spo2, 1)
        lay.addWidget(Separator())
        lay.addWidget(self.card_bpm, 1)
        lay.addWidget(Separator())

        # heart / perfusion row
        row = QtWidgets.QHBoxLayout()
        row.setContentsMargins(18, 0, 18, 0)
        self.lbl_heart = QtWidgets.QLabel("♥")
        self.lbl_heart.setStyleSheet(f"color:{NEON_DIM}; font-size:34px;")
        self.lbl_perf = QtWidgets.QLabel("PERF  --")
        self.lbl_perf.setStyleSheet(
            f"color:{NEON_DIM}; font-family:{MONO}; font-size:16px; letter-spacing:2px;")
        row.addWidget(self.lbl_heart)
        row.addStretch(1)
        row.addWidget(self.lbl_perf)
        lay.addLayout(row)

        # alarm banner
        self.lbl_alarm = QtWidgets.QLabel("")
        self.lbl_alarm.setWordWrap(True)
        self.lbl_alarm.setAlignment(QtCore.Qt.AlignCenter)
        self.lbl_alarm.setStyleSheet(
            f"color:{ALARM}; font-family:{MONO}; font-size:20px;"
            "font-weight:bold; letter-spacing:2px; padding:8px;")
        self.lbl_alarm.setFixedHeight(76)
        lay.addWidget(self.lbl_alarm)

        return panel

    # -----------------------------------------------------------------
    #  RENDER LOOP  (main thread only)
    # -----------------------------------------------------------------
    def _drain(self):
        """Pull everything the reader has produced since the last frame."""
        ir, red, bpm, spo2 = [], [], self.bpm, self.spo2
        q = self.queue
        for _ in range(4000):
            try:
                s = q.popleft()
            except IndexError:
                break
            ir.append(s[0]); red.append(s[1])
            if s[2] > 0: bpm = s[2]
            if s[3] > 0: spo2 = s[3]
        self.bpm, self.spo2 = bpm, spo2
        return np.asarray(ir), np.asarray(red)

    def _write(self, y):
        """Write a block into the ring buffer, wrapping at the right edge."""
        n = y.size
        if n == 0:
            return
        if n >= self.N:                                   # slower GUI than data
            y, n = y[-self.N:], self.N
        end = self.wr + n
        if end <= self.N:
            self.buf[self.wr:end] = y
        else:
            k = self.N - self.wr
            self.buf[self.wr:] = y[:k]
            self.buf[:end - self.N] = y[k:]
        self.wr = end % self.N

    def _tick(self):
        ir, _red = self._drain()

        if ir.size:
            self._rx_count += ir.size
            self.last_rx = time.perf_counter()

            # ---- finger presence (slow EMA on the raw DC level) ----------
            a = 0.15
            for v in (ir[0], ir[-1]):
                self.ir_ema = (1 - a) * self.ir_ema + a * v if self.ir_ema else v
            present = self.ir_ema > self.args.ir_threshold

            if present:
                if not self.finger:                       # fresh contact
                    self.bp.reset(ir[0])
                    self.gain, self.env_amp = 1.0, 0.0
                y = self.bp.process(ir)                   # 0.5-5 Hz band-pass
                if self.args.invert:
                    y = -y

                # adaptive gain: decaying peak tracker, so the tallest systolic
                # peak sits at ~0.9 of full scale instead of clipping the rail
                peak = float(np.max(np.abs(y))) if y.size else 0.0
                self.env_amp = max(peak, self.env_amp * 0.997)
                if self.env_amp > 1e-6:
                    target = 0.90 / self.env_amp
                    self.gain += 0.05 * (target - self.gain)
                yn = np.clip(y * self.gain, -1.1, 1.1)
                amp = self.env_amp

                if self.beat.feed(y):
                    self._beat_on = 8                     # frames of heart glow
                self.perf = amp
            else:
                self.bp.reset(self.ir_ema)
                yn = np.zeros_like(ir)                    # flatline to centre
                self.perf = 0.0

            self.finger = present
            self._write(yn)

        # ---- stale link -> keep sweeping a flatline ---------------------
        elif self.last_rx and (time.perf_counter() - self.last_rx) > 1.0:
            self.finger = False
            self._write(np.zeros(max(1, int(self.fs / GUI_FPS))))

        self._refresh_curves()
        self._refresh_numbers()

    def _refresh_curves(self):
        # blanking bar: erase the slice just ahead of the cursor
        buf = self.buf
        w = self.wr
        idx = (np.arange(w, w + self.blank) % self.N)
        buf[idx] = np.nan

        self.curve_body.setData(self.x, buf, connect="finite")

        lo = max(0, w - TRAIL_SAMPLES)                    # trail (no wrap needed)
        self.curve_mid.setData(self.x[lo:w], buf[lo:w], connect="finite")
        lo2 = max(0, w - TRAIL_SAMPLES // 4)
        self.curve_head.setData(self.x[lo2:w], buf[lo2:w], connect="finite")

        if w > 0 and np.isfinite(buf[w - 1]):
            self.dot.setData([w - 1], [buf[w - 1]])
        else:
            self.dot.setData([], [])

        self.eraser.setRegion([w, min(w + self.blank, self.N)])

    def _refresh_numbers(self):
        # heart beat glow
        if self._beat_on > 0:
            self._beat_on -= 1
            self.lbl_heart.setStyleSheet(f"color:{NEON}; font-size:38px;")
        else:
            self.lbl_heart.setStyleSheet(f"color:{NEON_DIM}; font-size:34px;")

        if not self.finger:
            self.card_spo2.set_value("--", dim=True)
            self.card_bpm.set_value("--", dim=True)
            self.lbl_perf.setText("PERF  --")
            msg = "SENSOR DISCONNECTED\nCHECK PROBE"
            self.lbl_alarm.setText(msg if self._blink else "")
            return

        s = int(round(self.spo2)) if self.spo2 > 0 else None
        b = int(round(self.bpm)) if self.bpm > 0 else None
        s_alarm = s is not None and s < SPO2_ALARM_LOW
        b_alarm = b is not None and not (BPM_ALARM_LOW <= b <= BPM_ALARM_HIGH)

        self.card_spo2.set_value(f"{s}" if s else "--",
                                 alarm=s_alarm and self._blink, dim=s is None)
        self.card_bpm.set_value(f"{b}" if b else "--",
                                alarm=b_alarm and self._blink, dim=b is None)
        self.lbl_perf.setText(f"PERF  {getattr(self, 'perf', 0.0):5.0f}")

        alarms = []
        if s_alarm: alarms.append("LOW SpO2")
        if b_alarm: alarms.append("PULSE OUT OF RANGE")
        self.lbl_alarm.setText(("  |  ".join(alarms) if self._blink else "") if alarms else "")

    # -----------------------------------------------------------------
    def _blink_tick(self):
        self._blink = not self._blink

    def _clock_tick(self):
        self.lbl_clock.setText(time.strftime("%H:%M:%S"))
        now = time.perf_counter()
        dt = now - self._rx_t0
        if dt > 0:
            self.sps = self._rx_count / dt
        self._rx_count, self._rx_t0 = 0, now
        base = self.lbl_link.text().split("   [")[0]
        self.lbl_link.setText(f"{base}   [{self.sps:5.1f} SPS]")

    def _on_status(self, msg, ok):
        self.lbl_link.setText(msg)
        self.lbl_link.setStyleSheet(
            f"color:{NEON if ok else AMBER}; font-family:{MONO};"
            "font-size:15px; letter-spacing:2px;")

    # -----------------------------------------------------------------
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
#  5.  ENTRY POINT
# ===========================================================================
def main():
    ap = argparse.ArgumentParser(description="Clinical PPG monitor for MAX30100")
    ap.add_argument("--port", default=None, help="COM5 | /dev/ttyUSB0 (default: auto)")
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--fs", type=float, default=FS, help="sensor sample rate (Hz)")
    ap.add_argument("--ir-threshold", type=float, default=IR_FINGER_THRESH,
                    dest="ir_threshold", help="raw IR level below which = no finger")
    ap.add_argument("--invert", action="store_true",
                    help="flip the pleth polarity (systolic peak up)")
    ap.add_argument("--simulate", action="store_true", help="synthetic PPG, no hardware")
    ap.add_argument("--windowed", action="store_true", help="do not start fullscreen")
    args = ap.parse_args()

    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(f"QWidget{{background:{BG};}}")

    win = MonitorWindow(args)
    win.showMaximized() if args.windowed else win.showFullScreen()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()