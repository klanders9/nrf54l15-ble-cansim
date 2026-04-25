"""
central.py — Raspberry Pi 4 BLE central for nRF54L15 telemetry.

Setup:
    pip install -r requirements.txt

Run:
    python3 central.py                  # rich terminal dashboard (default)
    python3 central.py --plot           # live matplotlib plot (IMU only)
    python3 central.py --csv my.csv     # custom CSV output path

Scans for "NordicTelemetry", subscribes to:
  - Project 1: accel + gyro notify characteristics (~50 Hz)
  - Project 2: EEC1 / ET1 / EFLP1 CAN gateway characteristics (notify-on-change)
"""

import argparse
import asyncio
import csv
import select
import struct
import sys
import termios
import threading
import time
import tty
from collections import deque

from bleak import BleakClient, BleakScanner
from rich.layout import Layout
from rich.live import Live
from rich.panel import Panel
from rich.table import Table

# ── BLE identifiers — must match firmware ──────────────────────────────────────
DEVICE_NAME = "NordicTelemetry"

# Project 1 — IMU (TelemetryService.cpp)
ACCEL_UUID = "b0b1b2b3-b4b5-b6b7-b8b9-babbbcbdbebf"
GYRO_UUID  = "c0c1c2c3-c4c5-c6c7-c8c9-cacbcccdcecf"

# Project 2 — CAN gateway (CanGatewayService.cpp)
EEC1_UUID  = "e0e1e2e3-e4e5-e6e7-e8e9-eaebecedeeef"  # PGN 61444 — engine RPM
ET1_UUID   = "f0f1f2f3-f4f5-f6f7-f8f9-fafbfcfdfeff"  # PGN 65262 — coolant temp
EFLP1_UUID = "10111213-1415-1617-1819-1a1b1c1d1e1f"  # PGN 65263 — fuel pressure

PLOT_WINDOW = 200  # samples in scrolling plot (--plot mode only)

# ── Shared state (BLE thread writes, display thread reads) ─────────────────────
_lock      = threading.Lock()
_latest    = dict(ax=0.0, ay=0.0, az=0.0, gx=0.0, gy=0.0, gz=0.0)
_buf       = {k: deque(maxlen=PLOT_WINDOW) for k in _latest}
_sample_ts: deque = deque(maxlen=100)  # gyro callback timestamps for Hz calc

_can_latest = dict(rpm=None, coolant_c=None, fuel_kpa=None, last_ts=None)

_csv_writer = None
_t0: float  = 0.0


# ── J1939 payload decoding ─────────────────────────────────────────────────────
# BLE payload (12 bytes): PGN[0-2 LE] | SA[3] | J1939 data[4-11]

def _decode_eec1(data: bytearray) -> float:
    # Engine Speed: J1939 data bytes 3-4 (payload offset 7), uint16 LE, 0.125 rpm/bit
    raw = struct.unpack_from('<H', data, 7)[0]
    return raw / 8.0

def _decode_et1(data: bytearray) -> float:
    # Coolant Temp: J1939 data byte 0 (payload offset 4), 1°C/bit, offset -40°C
    return float(data[4]) - 40.0

def _decode_eflp1(data: bytearray) -> float:
    # Fuel Delivery Pressure: J1939 data byte 0 (payload offset 4), 4 kPa/bit
    return float(data[4]) * 4.0


# ── Notification callbacks ─────────────────────────────────────────────────────

def _on_accel(sender, data: bytearray) -> None:
    ax, ay, az = (v / 100.0 for v in struct.unpack_from('<3h', data))
    with _lock:
        _latest.update(ax=ax, ay=ay, az=az)
        _buf['ax'].append(ax)
        _buf['ay'].append(ay)
        _buf['az'].append(az)


def _on_gyro(sender, data: bytearray) -> None:
    # Gyro arrives second (same work-queue tick as accel), so _latest has fresh accel.
    gx, gy, gz = (v / 100.0 for v in struct.unpack_from('<3h', data))
    ts = time.time()
    with _lock:
        _latest.update(gx=gx, gy=gy, gz=gz)
        _buf['gx'].append(gx)
        _buf['gy'].append(gy)
        _buf['gz'].append(gz)
        _sample_ts.append(ts)
        row = [f'{ts - _t0:.4f}',
               _latest['ax'], _latest['ay'], _latest['az'],
               gx, gy, gz]
    if _csv_writer:
        _csv_writer.writerow(row)


def _on_eec1(sender, data: bytearray) -> None:
    rpm = _decode_eec1(data)
    with _lock:
        _can_latest['rpm']     = rpm
        _can_latest['last_ts'] = time.time()


def _on_et1(sender, data: bytearray) -> None:
    coolant = _decode_et1(data)
    with _lock:
        _can_latest['coolant_c'] = coolant
        _can_latest['last_ts']   = time.time()


def _on_eflp1(sender, data: bytearray) -> None:
    fuel = _decode_eflp1(data)
    with _lock:
        _can_latest['fuel_kpa'] = fuel
        _can_latest['last_ts']  = time.time()


# ── BLE task ───────────────────────────────────────────────────────────────────

async def _ble_task(stop_event: threading.Event) -> None:
    print(f"Scanning for '{DEVICE_NAME}'...")
    device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    if device is None:
        print(f"  '{DEVICE_NAME}' not found — is it advertising?", file=sys.stderr)
        stop_event.set()
        return

    print(f"Found {device.address}  Connecting...")
    async with BleakClient(device) as client:
        await client.start_notify(ACCEL_UUID, _on_accel)
        await client.start_notify(GYRO_UUID,  _on_gyro)

        try:
            await client.start_notify(EEC1_UUID,  _on_eec1)
            await client.start_notify(ET1_UUID,   _on_et1)
            await client.start_notify(EFLP1_UUID, _on_eflp1)
        except Exception as exc:
            print(f"  CAN gateway not found ({exc}); IMU-only mode", file=sys.stderr)

        print("Subscribed. Dashboard active — press Ctrl-C to stop.\n")
        while not stop_event.is_set():
            await asyncio.sleep(0.1)
        await client.stop_notify(ACCEL_UUID)
        await client.stop_notify(GYRO_UUID)


def _run_ble(stop_event: threading.Event) -> None:
    asyncio.run(_ble_task(stop_event))


# ── Rich terminal dashboard ────────────────────────────────────────────────────

_IMU_ROWS = [
    ('ax', 'Accel X', 'm/s²'),
    ('ay', 'Accel Y', 'm/s²'),
    ('az', 'Accel Z', 'm/s²'),
    ('gx', 'Gyro  X', 'rad/s'),
    ('gy', 'Gyro  Y', 'rad/s'),
    ('gz', 'Gyro  Z', 'rad/s'),
]


def _imu_panel(snapshot: dict, rate: int) -> Panel:
    t = Table(show_header=False, box=None, padding=(0, 1))
    t.add_column(style='cyan',  width=10)
    t.add_column(style='green', justify='right', width=10)
    t.add_column(style='dim',   width=6)
    for key, label, unit in _IMU_ROWS:
        t.add_row(label, f'{snapshot[key]:+.3f}', unit)
    t.add_row('', '', '')
    t.add_row('Rate', str(rate), 'Hz')
    return Panel(t, title='[bold cyan]IMU Telemetry[/]', border_style='cyan')


def _can_panel(snap: dict) -> Panel:
    def _fmt(v, fmt: str) -> str:
        return format(v, fmt) if v is not None else '—'

    t = Table(show_header=False, box=None, padding=(0, 1))
    t.add_column(style='yellow',       width=16)
    t.add_column(style='bright_white', justify='right', width=8)
    t.add_column(style='dim',          width=5)
    t.add_row('Engine RPM',    _fmt(snap['rpm'],       '.0f'), 'RPM')
    t.add_row('Coolant Temp',  _fmt(snap['coolant_c'], '.0f'), '°C')
    t.add_row('Fuel Pressure', _fmt(snap['fuel_kpa'],  '.0f'), 'kPa')
    t.add_row('', '', '')
    last = snap['last_ts']
    if last is not None:
        t.add_row('Updated', f'{time.time() - last:.1f}', 's ago')
    else:
        t.add_row('Updated', '—', '')
    return Panel(t, title='[bold yellow]J1939 CAN Gateway[/]', border_style='yellow')


def _run_keyboard(stop_event: threading.Event) -> None:
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not stop_event.is_set():
            if select.select([sys.stdin], [], [], 0.2)[0]:
                if sys.stdin.read(1).lower() == 'q':
                    stop_event.set()
                    break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _run_dashboard(stop_event: threading.Event) -> None:
    layout = Layout()
    layout.split_row(Layout(name='imu'), Layout(name='can'))
    threading.Thread(target=_run_keyboard, args=(stop_event,), daemon=True).start()
    try:
        with Live(layout, refresh_per_second=10, screen=True):
            while not stop_event.is_set():
                with _lock:
                    imu_snap = dict(_latest)
                    ts_list  = list(_sample_ts)
                    can_snap = dict(_can_latest)
                now  = time.time()
                rate = len([t for t in ts_list if now - t < 1.0])
                layout['imu'].update(_imu_panel(imu_snap, rate))
                layout['can'].update(_can_panel(can_snap))
                time.sleep(0.1)
    except KeyboardInterrupt:
        stop_event.set()


# ── Live matplotlib plot (--plot mode, IMU only) ──────────────────────────────

def _run_plot(stop_event: threading.Event) -> None:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation

    fig, (ax_a, ax_g) = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    fig.suptitle("NordicTelemetry — live IMU data", fontsize=12)
    ax_a.set_ylabel("Accel (m/s²)")
    ax_a.set_ylim(-20, 20)
    ax_a.axhline(0, color='k', linewidth=0.5)
    ax_a.grid(True, alpha=0.3)
    ax_g.set_ylabel("Gyro (rad/s)")
    ax_g.set_ylim(-10, 10)
    ax_g.set_xlabel(f"newest {PLOT_WINDOW} samples")
    ax_g.axhline(0, color='k', linewidth=0.5)
    ax_g.grid(True, alpha=0.3)

    x        = list(range(PLOT_WINDOW))
    nan_fill = [float('nan')] * PLOT_WINDOW
    lines = {
        'ax': ax_a.plot(x, nan_fill, label='ax')[0],
        'ay': ax_a.plot(x, nan_fill, label='ay')[0],
        'az': ax_a.plot(x, nan_fill, label='az')[0],
        'gx': ax_g.plot(x, nan_fill, label='gx')[0],
        'gy': ax_g.plot(x, nan_fill, label='gy')[0],
        'gz': ax_g.plot(x, nan_fill, label='gz')[0],
    }
    ax_a.legend(loc='upper right', fontsize=8)
    ax_g.legend(loc='upper right', fontsize=8)
    plt.tight_layout()

    def update(_frame):
        if stop_event.is_set():
            plt.close('all')
            return list(lines.values())
        with _lock:
            snapshot = {k: list(v) for k, v in _buf.items()}
        for key, line in lines.items():
            d      = snapshot[key]
            padded = [float('nan')] * (PLOT_WINDOW - len(d)) + d
            line.set_ydata(padded)
        return list(lines.values())

    _anim = animation.FuncAnimation(  # noqa: F841 — reference kept to prevent GC
        fig, update, interval=100, blit=True, cache_frame_data=False,
    )
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="nRF54L15 BLE telemetry dashboard")
    parser.add_argument('--csv', default='imu_log.csv', metavar='FILE',
                        help='CSV output path (default: imu_log.csv)')
    parser.add_argument('--plot', action='store_true',
                        help='Live matplotlib plot instead of terminal dashboard '
                             '(IMU only; better on desktop than on Pi 4)')
    args = parser.parse_args()

    global _csv_writer, _t0
    _t0 = time.time()

    csv_fh = open(args.csv, 'w', newline='')
    _csv_writer = csv.writer(csv_fh)
    _csv_writer.writerow(['timestamp_s', 'ax', 'ay', 'az', 'gx', 'gy', 'gz'])

    stop_event = threading.Event()
    ble_thread = threading.Thread(target=_run_ble, args=(stop_event,), daemon=True)
    ble_thread.start()

    try:
        if args.plot:
            _run_plot(stop_event)
        else:
            _run_dashboard(stop_event)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        ble_thread.join(timeout=3.0)
        csv_fh.close()
        print(f"\nSaved to {args.csv}")


if __name__ == '__main__':
    main()
