# nRF54L15 BLE Telemetry + CAN Gateway

Two connected BLE peripheral projects on the Nordic nRF54L15 using the nRF Connect SDK.

**Project 1 — IMU Telemetry:** The nRF54L15 reads accelerometer and gyroscope data from an LSM6DS3TR-C IMU over I2C and streams it to a host via a custom GATT service. The host logs data to CSV and displays a live terminal dashboard.

**Project 2 — J1939 CAN Gateway Simulator:** A second GATT service simulates a J1939 CAN bus in software — engine RPM, coolant temperature, and fuel delivery pressure drift realistically with noise and are exposed as notify-on-change BLE characteristics, mirroring the behavior of a real telematics dongle.

---

## Hardware

| Component | Role |
|---|---|
| [Nordic nRF54L15 DK](https://www.nordicsemi.com/Products/Development-hardware/nRF54L15-DK) | BLE peripheral, IMU host |
| [Adafruit LSM6DS3TR-C breakout](https://www.adafruit.com/product/4503) | I2C IMU (accel + gyro) |
| Raspberry Pi 4 | BLE central, data logger |

**Wiring (LSM6DS3TR-C → nRF54L15 DK):**

| Breakout | nRF54L15 DK |
|---|---|
| 3V | 3.3V |
| GND | GND |
| SDA | P1.12 |
| SCL | P1.11 |

> The LSM6DS3TR-C is register-compatible with the ST LSM6DSL. Zephyr has no dedicated driver for it; the `st,lsm6dsl` compatible string is used in the devicetree overlay.
>
> Default I2C address is `0x6a` (SA0 low). If your breakout pulls SA0 high, change the address in `boards/nrf54l15dk_nrf54l15_cpuapp.overlay` to `0x6b`.

---

## Prerequisites

### Firmware (nRF54L15 DK)

- [nRF Connect SDK v3.2.4](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/3.2.4/nrf/getting_started.html) with its toolchain (installed via nRF Connect for Desktop → Toolchain Manager)
- [nRF Connect for VS Code](https://marketplace.visualstudio.com/items?itemName=nordic-semiconductor.nrf-connect) extension (optional but recommended)

### Pi 4 central

- Python 3.9+
- A Bluetooth adapter (the Pi 4's built-in adapter works)

```
pip install -r host/requirements.txt
```

---

## Build & Flash

Using the VS Code extension: open the repo as an nRF Connect application, select board `nrf54l15dk/nrf54l15/cpuapp`, and click **Build** → **Flash**.

Using `west` directly (from the repo root, with the nRF Connect SDK environment active):

```bash
west build -b nrf54l15dk/nrf54l15/cpuapp
west flash
```

Once flashed, the board advertises as **NordicTelemetry**.

---

## Running the Pi Central

```bash
# Terminal dashboard (default — works well on Pi 4)
python3 host/central.py

# Live scrolling plot (better on a desktop with a GPU)
python3 host/central.py --plot

# Custom CSV output path
python3 host/central.py --csv my_run.csv
```

The script scans for `NordicTelemetry` and subscribes to all available GATT notify characteristics. If the CAN gateway service is present, a second panel shows the J1939 values alongside the IMU panel. IMU data is written to CSV at ~50 Hz. Press `q` or Ctrl-C to quit.

---

## Project Structure

```
├── CMakeLists.txt
├── prj.conf                          # Zephyr Kconfig
├── boards/
│   └── nrf54l15dk_nrf54l15_cpuapp.overlay   # I2C + LSM6DS3TR-C devicetree
├── src/
│   ├── main.cpp
│   ├── sensor/
│   │   ├── ImuSensor.hpp             # C++ wrapper around Zephyr sensor API
│   │   └── ImuSensor.cpp
│   ├── ble/
│   │   ├── TelemetryService.hpp      # Project 1: IMU GATT service
│   │   └── TelemetryService.cpp
│   └── can/
│       ├── CanSimulator.hpp          # Project 2: J1939 frame simulator
│       ├── CanSimulator.cpp
│       ├── CanGatewayService.hpp     # Project 2: CAN gateway GATT service
│       └── CanGatewayService.cpp
└── host/
    ├── central.py                    # BLE central — IMU + CAN dashboard
    └── requirements.txt
```

---

## GATT Services

### Project 1 — IMU Telemetry (`TelemetryService`)

| | UUID |
|---|---|
| Service | `a0a1a2a3-a4a5-a6a7-a8a9-aaabacadaeaf` |
| Accel XYZ | `b0b1b2b3-b4b5-b6b7-b8b9-babbbcbdbebf` |
| Gyro XYZ | `c0c1c2c3-c4c5-c6c7-c8c9-cacbcccdcecf` |

Both characteristics notify at 50 Hz. Payload is 6 bytes: 3× `int16` little-endian, units 0.01 m/s² (accel) and 0.01 rad/s (gyro).

### Project 2 — CAN Gateway (`CanGatewayService`)

| | UUID | J1939 PGN |
|---|---|---|
| Service | `d0d1d2d3-d4d5-d6d7-d8d9-dadbdcdddedf` | — |
| Engine RPM (EEC1) | `e0e1e2e3-e4e5-e6e7-e8e9-eaebecedeeef` | 61444 |
| Coolant Temp (ET1) | `f0f1f2f3-f4f5-f6f7-f8f9-fafbfcfdfeff` | 65262 |
| Fuel Pressure (EFLP1) | `10111213-1415-1617-1819-1a1b1c1d1e1f` | 65263 |

Characteristics notify on value change (not on a fixed timer). Payload is 12 bytes: `PGN[3 LE] | src_addr[1] | J1939 data[8]`. Change thresholds: ±25 RPM for EEC1, any 1°C change for ET1, ±12 kPa for EFLP1.

The simulated values drift with low-pass filtering and random noise to mimic a real idling engine. Fuel pressure is loosely coupled to RPM.

---

## Planned

**Project 2 — Phase 4 (optional): real CAN hardware**

Replace the software simulator with an MCP2515 SPI CAN controller (~$10) wired to the DK's SPI bus to receive actual CAN traffic. `CanSimulator` can be swapped out for a real driver without changing `CanGatewayService`.

---

## License

MIT — see [LICENSE](LICENSE).
