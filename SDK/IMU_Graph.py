#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PiEEG_XR  -  live accelerometer + gyroscope plot
=================================================

Reads the BLE telegram and plots the LSM6DS3 IMU, which lives at the START
of the telegram (matches your p2p_server_app.c):

    local[0]     = LSM6DS3 status byte
    local[1..6]  = accel  X,Y,Z   (int16 little-endian: low byte then high)
    local[7..12] = gyro   X,Y,Z   (int16 little-endian)

Config in firmware: set_speed = 0x60 -> 416 Hz, accel +/-2 g, gyro 245 dps.

Requirements  (run from a TERMINAL, not IDLE):
    pip install bleak matplotlib
    python pieeg_xr_imu_plot.py
Options:
    --address AA:BB:CC:DD:EE:FF   connect by address (skip name scan)
    --window 500                 samples shown in the rolling window
"""

import sys

print("PiEEG_XR IMU plot starting ...", flush=True)
if sys.version_info < (3, 8):
    sys.exit("Need Python 3.8+ (3.11/3.12 recommended). bleak will not run on older.")

import argparse
import asyncio
import struct
import threading
from collections import deque

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    sys.exit("bleak not installed. Run:  python -m pip install bleak")
try:
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:
    sys.exit("matplotlib not installed. Run:  python -m pip install matplotlib")

# ---------------------------------------------------------------------------
DEVICE_NAME = "PiEEG_XR"
NOTIFY_UUID = "0000fe42-8e22-4541-9d4c-21edae82ed19"

# telegram byte offsets (from firmware)
STATUS_OFF = 0
ACC_OFF    = 1     # accel X,Y,Z int16 LE -> bytes 1..6
GYR_OFF    = 7     # gyro  X,Y,Z int16 LE -> bytes 7..12
IMU_MIN_LEN = 13   # need at least status + 12 IMU bytes

# LSM6DS3 sensitivities for FS = +/-2 g and 245 dps
ACC_G_PER_LSB   = 0.061 / 1000.0     # 0.061 mg/LSB
GYR_DPS_PER_LSB = 8.75  / 1000.0     # 8.75 mdps/LSB

# ---------------------------------------------------------------------------
# Shared rolling buffers (written by BLE thread, read by the plot on main thread)
# ---------------------------------------------------------------------------
class Buffers:
    def __init__(self, window):
        self.n = 0
        self.idx = deque(maxlen=window)
        self.ax = deque(maxlen=window)
        self.ay = deque(maxlen=window)
        self.az = deque(maxlen=window)
        self.gx = deque(maxlen=window)
        self.gy = deque(maxlen=window)
        self.gz = deque(maxlen=window)
        self.count = 0
        self.connected = False

    def push(self, a, g):
        self.n += 1
        self.count += 1
        self.idx.append(self.n)
        self.ax.append(a[0]); self.ay.append(a[1]); self.az.append(a[2])
        self.gx.append(g[0]); self.gy.append(g[1]); self.gz.append(g[2])


def decode_imu(pkt):
    if len(pkt) < IMU_MIN_LEN:
        return None
    ax, ay, az = struct.unpack_from("<hhh", pkt, ACC_OFF)
    gx, gy, gz = struct.unpack_from("<hhh", pkt, GYR_OFF)
    a = (ax * ACC_G_PER_LSB, ay * ACC_G_PER_LSB, az * ACC_G_PER_LSB)
    g = (gx * GYR_DPS_PER_LSB, gy * GYR_DPS_PER_LSB, gz * GYR_DPS_PER_LSB)
    return a, g


# ---------------------------------------------------------------------------
# BLE runs in its own thread with its own asyncio loop
# ---------------------------------------------------------------------------
async def ble_main(buf, address, name, stop_event):
    if address:
        print(f"Scanning for {address} ...", flush=True)
        dev = await BleakScanner.find_device_by_address(address, timeout=10.0)
    else:
        print(f"Scanning for '{name}' ...", flush=True)
        dev = await BleakScanner.find_device_by_filter(
            lambda d, ad: (ad.local_name == name) or (d.name == name), timeout=10.0
        )
    if dev is None:
        print("Device not found. Is it advertising and not connected elsewhere?")
        stop_event.set()
        return

    print(f"Found {dev.address}. Connecting ...", flush=True)

    def handler(_char, data):
        r = decode_imu(data)
        if r is not None:
            buf.push(r[0], r[1])

    async with BleakClient(dev) as client:
        buf.connected = client.is_connected
        print(f"Connected: {client.is_connected}. Streaming IMU ...", flush=True)
        await client.start_notify(NOTIFY_UUID, handler)
        try:
            while not stop_event.is_set():
                await asyncio.sleep(0.2)
        finally:
            try:
                await client.stop_notify(NOTIFY_UUID)
            except Exception:
                pass
    print("BLE disconnected.", flush=True)


def start_ble_thread(buf, address, name, stop_event):
    def runner():
        if sys.platform == "win32":
            try:
                asyncio.set_event_loop_policy(asyncio.WindowsProactorEventLoopPolicy())
            except Exception:
                pass
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(ble_main(buf, address, name, stop_event))
        except Exception as e:
            print(f"BLE thread error: {e}", flush=True)
            stop_event.set()

    t = threading.Thread(target=runner, daemon=True)
    t.start()
    return t


# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(description="Live accel + gyro plot for PiEEG_XR")
    p.add_argument("--address", help="BLE address (skip name scan)")
    p.add_argument("--name", default=DEVICE_NAME)
    p.add_argument("--window", type=int, default=500, help="samples in rolling window")
    args = p.parse_args()

    buf = Buffers(args.window)
    stop_event = threading.Event()
    start_ble_thread(buf, args.address, args.name, stop_event)

    # --- figure: accel (top) + gyro (bottom) ---
    fig, (ax_acc, ax_gyr) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig.suptitle("PiEEG_XR  -  LSM6DS3 IMU (live)")

    (la_x,) = ax_acc.plot([], [], label="acc X", lw=1)
    (la_y,) = ax_acc.plot([], [], label="acc Y", lw=1)
    (la_z,) = ax_acc.plot([], [], label="acc Z", lw=1)
    ax_acc.set_ylabel("accel (g)")
    ax_acc.set_ylim(-2.2, 2.2)
    ax_acc.legend(loc="upper right", ncol=3)
    ax_acc.grid(True, alpha=0.3)

    (lg_x,) = ax_gyr.plot([], [], label="gyro X", lw=1)
    (lg_y,) = ax_gyr.plot([], [], label="gyro Y", lw=1)
    (lg_z,) = ax_gyr.plot([], [], label="gyro Z", lw=1)
    ax_gyr.set_ylabel("gyro (dps)")
    ax_gyr.set_xlabel("sample")
    ax_gyr.set_ylim(-260, 260)
    ax_gyr.legend(loc="upper right", ncol=3)
    ax_gyr.grid(True, alpha=0.3)

    def update(_frame):
        if not buf.idx:
            return la_x, la_y, la_z, lg_x, lg_y, lg_z
        x = list(buf.idx)
        la_x.set_data(x, list(buf.ax))
        la_y.set_data(x, list(buf.ay))
        la_z.set_data(x, list(buf.az))
        lg_x.set_data(x, list(buf.gx))
        lg_y.set_data(x, list(buf.gy))
        lg_z.set_data(x, list(buf.gz))
        ax_acc.set_xlim(x[0], x[-1] if x[-1] > x[0] else x[0] + 1)
        # auto-fit y a little beyond the data, but keep sane floor ranges
        def fit(axis, series, floor):
            vals = [v for s in series for v in s]
            if vals:
                lo, hi = min(vals), max(vals)
                pad = max((hi - lo) * 0.15, floor)
                axis.set_ylim(lo - pad, hi + pad)
        fit(ax_acc, [buf.ax, buf.ay, buf.az], 0.2)
        fit(ax_gyr, [buf.gx, buf.gy, buf.gz], 5.0)
        ax_acc.set_title(f"samples: {buf.count}    (accel magnitude at rest should be ~1 g)")
        return la_x, la_y, la_z, lg_x, lg_y, lg_z

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

    try:
        plt.show()          # blocks on the main thread until the window closes
    finally:
        stop_event.set()

    print("Closed.")


if __name__ == "__main__":
    main()
