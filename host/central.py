"""
central.py — Raspberry Pi 4 BLE central for nRF54L15 IMU telemetry.

Setup:
    pip install -r requirements.txt

Run:
    python3 central.py                  # curses terminal dashboard (default)
    python3 central.py --plot           # live matplotlib plot (better on desktop)
    python3 central.py --csv my.csv     # custom CSV output path

Scans for "NordicTelemetry", subscribes to accel + gyro GATT notify
characteristics, and writes one timestamped CSV row per firmware tick (~50 Hz).
"""

import argparse
import asyncio
import csv
import curses
import struct
import sys
import threading
import time
from collections import deque

from bleak import BleakClient, BleakScanner

# ── BLE identifiers — must match TelemetryService.cpp ─────────────────────
DEVICE_NAME = "NordicTelemetry"
ACCEL_UUID  = "b0b1b2b3-b4b5-b6b7-b8b9-babbbcbdbebf"
GYRO_UUID   = "c0c1c2c3-c4c5-c6c7-c8c9-cacbcccdcecf"

PLOT_WINDOW = 200  # samples in the scrolling plot (--plot mode only)

# ── Shared state (asyncio BLE thread writes, display thread reads) ─────────
_lock      = threading.Lock()
_latest    = dict(ax=0.0, ay=0.0, az=0.0, gx=0.0, gy=0.0, gz=0.0)
_buf       = {k: deque(maxlen=PLOT_WINDOW) for k in _latest}
_sample_ts: deque = deque(maxlen=100)  # timestamps of recent gyro callbacks for rate calc

_csv_writer = None
_t0: float  = 0.0


# ── Notification callbacks ─────────────────────────────────────────────────

def _on_accel(sender, data: bytearray) -> None:
    ax, ay, az = (v / 100.0 for v in struct.unpack_from('<3h', data))
    with _lock:
        _latest['ax'] = ax
        _latest['ay'] = ay
        _latest['az'] = az
        _buf['ax'].append(ax)
        _buf['ay'].append(ay)
        _buf['az'].append(az)


def _on_gyro(sender, data: bytearray) -> None:
    # Gyro arrives second (same work-queue tick as accel), so _latest has fresh accel.
    gx, gy, gz = (v / 100.0 for v in struct.unpack_from('<3h', data))
    ts = time.time()
    with _lock:
        _latest['gx'] = gx
        _latest['gy'] = gy
        _latest['gz'] = gz
        _buf['gx'].append(gx)
        _buf['gy'].append(gy)
        _buf['gz'].append(gz)
        _sample_ts.append(ts)
        row = [f'{ts - _t0:.4f}',
               _latest['ax'], _latest['ay'], _latest['az'],
               gx, gy, gz]
    if _csv_writer:
        _csv_writer.writerow(row)


# ── BLE task ───────────────────────────────────────────────────────────────

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
        print("Subscribed. Logging — press Ctrl-C to stop.\n")
        while not stop_event.is_set():
            await asyncio.sleep(0.1)
        await client.stop_notify(ACCEL_UUID)
        await client.stop_notify(GYRO_UUID)


def _run_ble(stop_event: threading.Event) -> None:
    asyncio.run(_ble_task(stop_event))


# ── Curses terminal dashboard ──────────────────────────────────────────────

def _run_curses(stdscr, stop_event: threading.Event) -> None:
    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_CYAN,  -1)  # headers
    curses.init_pair(2, curses.COLOR_GREEN, -1)  # values
    curses.init_pair(3, curses.COLOR_YELLOW, -1) # rate

    CHANNELS = [
        ('ax', 'Accel X', 'm/s²'),
        ('ay', 'Accel Y', 'm/s²'),
        ('az', 'Accel Z', 'm/s²'),
        ('gx', 'Gyro  X', 'rad/s'),
        ('gy', 'Gyro  Y', 'rad/s'),
        ('gz', 'Gyro  Z', 'rad/s'),
    ]

    while not stop_event.is_set():
        key = stdscr.getch()
        if key == ord('q'):
            stop_event.set()
            break

        with _lock:
            snapshot = dict(_latest)
            ts_list  = list(_sample_ts)

        now = time.time()
        recent = [t for t in ts_list if now - t < 1.0]
        rate = len(recent)

        stdscr.erase()
        h, w = stdscr.getmaxyx()
        title = "NordicTelemetry — IMU live data"
        stdscr.addstr(0, max(0, (w - len(title)) // 2), title,
                      curses.color_pair(1) | curses.A_BOLD)
        stdscr.addstr(1, 0, "─" * min(w - 1, 50), curses.color_pair(1))

        for i, (key_name, label, unit) in enumerate(CHANNELS):
            row = 3 + i
            val = snapshot[key_name]
            stdscr.addstr(row, 2,  f"{label}:", curses.color_pair(1))
            stdscr.addstr(row, 14, f"{val:+8.3f} {unit}", curses.color_pair(2))

        stdscr.addstr(10, 2, "─" * min(w - 3, 46), curses.color_pair(1))
        stdscr.addstr(11, 2, f"Rate: {rate:3d} Hz", curses.color_pair(3))
        stdscr.addstr(12, 2, "q — quit", curses.color_pair(1))

        stdscr.refresh()
        time.sleep(0.1)


def _run_dashboard(stop_event: threading.Event) -> None:
    try:
        curses.wrapper(_run_curses, stop_event)
    except KeyboardInterrupt:
        stop_event.set()


# ── Live matplotlib plot (--plot mode) ────────────────────────────────────

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
            d = snapshot[key]
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


# ── Entry point ────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="nRF54L15 BLE IMU logger")
    parser.add_argument('--csv', default='imu_log.csv', metavar='FILE',
                        help='CSV output path (default: imu_log.csv)')
    parser.add_argument('--plot', action='store_true',
                        help='Show live matplotlib plot instead of terminal dashboard '
                             '(better on desktop; slow on Pi 4)')
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
