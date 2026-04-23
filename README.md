# nRF54L15 BLE IMU Telemetry

A hands-on project exploring BLE peripheral development on the Nordic nRF54L15 using the nRF Connect SDK. The nRF54L15 reads accelerometer and gyroscope data from an LSM6DS3TR-C IMU over I2C and streams it to a Raspberry Pi 4 via a custom GATT service. The Pi logs data to CSV and displays a live terminal dashboard.

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

The script scans for `NordicTelemetry`, subscribes to the accel and gyro GATT notify characteristics, and writes one timestamped row to CSV per firmware timer tick (~50 Hz). The terminal dashboard shows all 6 channels and the incoming sample rate. Press `q` to quit.

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
│   └── ble/
│       ├── TelemetryService.hpp      # C++ wrapper around Zephyr GATT API
│       └── TelemetryService.cpp
└── host/
    ├── central.py                    # Raspberry Pi BLE central
    └── requirements.txt
```

---

## Planned

**Project 2 — BLE-to-CAN Gateway Simulator**

Simulate a J1939 CAN stream on the nRF54L15 (software-only to start, no extra hardware) and expose key parameters — engine RPM, coolant temp, fuel economy — over a second GATT service. The Pi central will display a live terminal dashboard using Python `rich`. Optional upgrade: add an MCP2515 SPI CAN controller to receive real CAN traffic.

---

## License

MIT — see [LICENSE](LICENSE).
