# ECG Monitor — AD8232 + Arduino + Python

A real-time ECG (electrocardiogram) monitoring system built with an AD8232 single-lead heart rate sensor, an Arduino, and a Python-based visualization engine. The goal was to move beyond a simple serial plotter and build something that looks and behaves like an actual clinical bedside monitor — dark UI, sweeping waveform, live filtering, and live BPM.

## Overview

The Arduino reads raw analog voltage from the AD8232 and streams it over serial. Python ingests that stream, cleans it up with a digital signal processing pipeline, calculates heart rate in real time, and renders it all on a hospital-monitor-style dashboard.

## Features

- **Clinical-style dashboard UI** — pitch-black background, ECG-paper-style grid, and a phosphor-green sweeping waveform with a fading trail effect (mimics real bedside monitors instead of a scrolling stock-ticker style plot)
- **Non-scrolling sweep display** — waveform draws left to right across a fixed 4–5 second window with a wiping "erase bar," preserving true heartbeat shape instead of distorting it like a scrolling chart
- **Live digital signal processing** — a three-stage filter chain cleans the raw signal before it's ever drawn:
  - High-pass filter (~0.5 Hz) to remove baseline wander from breathing/movement
  - Low-pass filter (~40–150 Hz) to smooth high-frequency muscle noise
  - Notch filter (50/60 Hz) to eliminate AC power line hum
- **Real-time BPM calculation** — dynamic peak-detection algorithm (simplified Pan-Tompkins style) using a rolling threshold, with beat-to-beat R-to-R interval timing so the BPM readout updates on every heartbeat rather than once per minute
- **Leads-off detection** — flashes a warning if the Arduino reports a flatline/disconnected electrode
- **System diagnostics readout** — live sampling rate and data drop rate
- **Dual-thread architecture** — a background thread handles serial ingestion into a thread-safe queue, while the main thread runs the UI at a locked 60 FPS, so filtering and rendering never block data collection

## Hardware

- Arduino (Uno/Nano or compatible)
- AD8232 single-lead ECG sensor module
- ECG electrode pads + leads
- USB cable for serial communication

## Software

- **Arduino IDE** — firmware that samples the AD8232 output and streams it over serial
- **Python** — signal processing and the real-time visualization dashboard (built on PyQtGraph rather than Matplotlib, since Matplotlib isn't fast enough for smooth real-time, high-frame-rate rendering)

## How It Works

1. **Arduino** samples the analog ECG signal from the AD8232 and sends raw integer values over serial.
2. **Ingestion thread (Python)** continuously reads the serial buffer and pushes raw samples into a FIFO queue.
3. **Filter pipeline** processes each sample: high-pass → low-pass → notch filter, in sequence.
4. **Peak detection** tracks the cleaned waveform, dynamically adjusting its detection threshold based on the average of the last few beats, and flags an R-spike whenever the signal crosses it.
5. **BPM calculation** measures the time between consecutive R-spikes and converts it to beats per minute: `BPM = 60 / (time between peaks in seconds)`.
6. **Rendering loop** draws the cleaned waveform, current BPM, lead status, and diagnostics onto the dashboard at 60 FPS.

## Setup

1. Wire the AD8232 to the Arduino (RA/LA/RL leads to electrodes, output to an analog pin).
2. Upload the Arduino sketch via Arduino IDE.
3. Install Python dependencies (e.g. `pyserial`, `pyqtgraph`, `numpy`/`scipy` for filtering).
4. Run the Python script and select the correct serial port.

## Notes

This project was built as a hands-on exploration of biomedical signal acquisition and real-time DSP — treating a hobby-grade sensor with the same processing pipeline (filtering, dynamic thresholding, real-time BPM) used in real clinical monitoring equipment.
