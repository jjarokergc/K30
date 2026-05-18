# CO2Meter Example Code

## Overview

This repository contains example/demo code for the CO2Meter CM-200 Sensor Development Board. It provides sample code for interfacing with CO2 and O2 gas sensors from multiple manufacturers (Senseair, GSS, SST/LuminOx, Cubic) using Arduino and Raspberry Pi platforms.

**Languages:** Arduino C++ (`.ino` sketches), Python 3 (Raspberry Pi scripts)

## Cursor Cloud specific instructions

### Repository structure

- `Sensair/` — Senseair K-Series & S8 CO2 sensors (UART & I2C examples for Arduino + Raspberry Pi)
- `GSS/` — GSS CozIR/MinIR/SprintIR/ExplorIR CO2 sensors (UART examples)
- `LuminOx/` — SST LuminOx O2 sensors (UART examples)
- `Cubic/` — Cubic NDIR CO2 sensors (UART Arduino example)
- `SE-11/` — Senseair Sunrise CO2 sensors (I2C & UART Arduino examples)
- `STM32/` — Git submodule (STM32WB55 firmware, not initialized by default)

### Development environment

- **Python scripts** require `pyserial` and `smbus3`. Install with: `pip3 install pyserial smbus3`
- **Arduino sketches** require `arduino-cli` with the `arduino:avr` core installed. All sketches target `arduino:avr:uno`.
- No web services, databases, or Docker containers are needed.
- `Sensair-USB.py` uses Python 2 syntax (`print` statement without parentheses); all other `.py` files are Python 3.

### Lint / Compile / Test

- **Python lint:** `python3 -m py_compile <file.py>` (no formal linter configured)
- **Arduino compile:** `arduino-cli compile --fqbn arduino:avr:uno <sketch_directory>/`
- **No automated test suite exists.** Actual execution requires physical sensor hardware (CM-200 board + gas sensor).
- To simulate sensor communication without hardware, use Python's `pty` module to create virtual serial ports and mock the UART protocol.

### Key caveats

- The I2C scripts (`smbus3`) will fail to open `/dev/i2c-*` without actual I2C hardware or kernel modules.
- Serial port paths are hardcoded (e.g., `/dev/ttyS0`, `/dev/ttyUSB0`). In a cloud environment, use virtual PTYs for testing.
- Two git submodules are declared in `.gitmodules` but may not be initialized. Use `git submodule update --init --recursive` if needed.
