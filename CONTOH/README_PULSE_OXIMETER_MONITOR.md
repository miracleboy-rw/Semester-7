# PPG Patient Monitor — MAX30100 + PyQt5

Real-time pulse oximetry and heart-rate monitor built around a MAX30100 sensor on an Arduino, visualized with a clinical patient-monitor-style desktop app (PyQt5 + pyqtgraph).

The Arduino is a dumb data-gathering node: it streams raw IR/RED samples plus a rough BPM/SpO2 estimate over serial. All real signal processing (bandpass filtering, gain, beat detection for display) and rendering happens on the PC.

![Monitor preview](monitor_preview.png)

## Features

- Pitch-black, high-contrast neon-cyan clinical aesthetic, 70/30 waveform-to-readout layout
- Non-scrolling **sweep** waveform (like a real bedside monitor) with an erasing blank bar and a fading trail, instead of a scrolling ticker
- Live **0.5–5 Hz Butterworth bandpass** filtering (`scipy.signal`, stateful SOS filtering) to strip baseline wander and high-frequency noise from the raw PPG
- **Dual-thread architecture**: a `QThread` handles all serial I/O and pushes samples into a thread-safe `deque`; the GUI thread only drains the queue, filters, and redraws at 60 FPS
- Finger-off detection with a flashing `SENSOR DISCONNECTED` alarm and waveform flatline
- Automatic serial reconnect on USB disconnects or corrupted packets — the app never crashes on a bad read
- `--simulate` mode: runs a synthetic PPG waveform with no hardware attached

## Repository contents

| File | Purpose |
|---|---|
| `clinical_monitor.py` | The PyQt5/pyqtgraph desktop application |
| `max30100_stream/max30100_stream.ino` | Main Arduino sketch (MAX30100lib) |
| `max30102_stream/max30102_stream.ino` | Drop-in alternative sketch for MAX30102 boards |
| `00_serial_test/00_serial_test.ino` | Diagnostic: confirms USB/serial link with no I2C involved |
| `01_i2c_scan/01_i2c_scan.ino` | Diagnostic: I2C bus scanner + idle line-level check |

## Hardware

- Arduino Uno/Nano (or similar 5V AVR board)
- MAX30100 breakout, I2C
- Wiring: `VIN`→see note below, `GND`→GND, `SDA`→A4, `SCL`→A5

**Common breakout issue:** many GY-MAX30100 boards pull SDA/SCL up to their internal 1.8V rail while a 5V Arduino pulls up to 5V. The two fight and the bus idles around ~2V, which reads as a logic LOW and hangs `Wire.begin()`. If `01_i2c_scan.ino` reports `SDA=0 SCL=0` or the device isn't found at `0x57`, add external **1kΩ pull-ups from SDA and SCL to 3.3V** and move the module's `VIN` to 3.3V. Diagnose with a multimeter: idle SDA-to-GND around 1.8–2.2V confirms this issue; near 0V indicates a short/dead module instead.

## Serial protocol

115200 baud, one ASCII line per sample:

```
<rawIR>,<rawRED>,<bpm>,<spo2>\n
```

Lines starting with `#` are treated as comments/status and ignored by the parser.

## Installation

```bash
pip install PyQt5 pyqtgraph numpy scipy pyserial
```

Flash `max30100_stream/max30100_stream.ino` (requires **MAX30100lib** by OXullo Intersecans, via Library Manager).

## Usage

```bash
python clinical_monitor.py --port COM5          # or /dev/ttyUSB0; omit to auto-detect
python clinical_monitor.py --simulate           # demo mode, no hardware needed
```

Useful flags:

| Flag | Description |
|---|---|
| `--port` | Serial port (default: auto-detect) |
| `--fs` | Sensor sample rate in Hz (default 100) |
| `--ir-threshold` | Raw IR level below which "no finger" is declared (default 5000; MAX30102 needs a higher value, e.g. 20000) |
| `--invert` | Flip pleth polarity if the waveform displays upside down |
| `--simulate` | Synthetic waveform, no hardware required |
| `--windowed` | Start maximized instead of fullscreen |

Keys: `F11`/`F` toggle fullscreen, `Esc` exit fullscreen, `Q` quit.

## Disclaimer

This is a hobby/educational project, not a medical device. The SpO2 estimate uses an uncalibrated empirical formula (`110 − 25·R`) and should not be trusted for any health decision.
