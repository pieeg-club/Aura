#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PiEEG_XR
109-byte BLE packet

Packet:
    0..12    IMU
    13..36   ADS1299 sample 1
    37..60   ADS1299 sample 2
    61..84   ADS1299 sample 3
    85..108  ADS1299 sample 4

Display:
    LEFT  = 3D IMU orientation
    RIGHT = 8 EMG channels in microvolts

Install:
    pip install bleak numpy matplotlib

Press R to make current orientation zero.
"""

import math
import time
import struct
import asyncio
import argparse
import threading
from pathlib import Path

from collections import deque

import numpy as np
import matplotlib.pyplot as plt

from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from bleak import BleakClient, BleakScanner


# ============================================================
# BLE
# ============================================================

DEVICE_NAME = "PiEEG_XR"

NOTIFY_UUID = "0000fe42-8e22-4541-9d4c-21edae82ed19"

PACKET_SIZE = 109


# ============================================================
# IMU
# ============================================================

IMU_SIZE = 13

ACC_OFFSET = 1
GYRO_OFFSET = 7

# LSM6DS3 ±2g
ACC_G_PER_LSB = 0.061 / 1000.0

# LSM6DS3 ±245 dps
GYRO_DPS_PER_LSB = 8.75 / 1000.0


# ============================================================
# ORIENTATION FILTER
# ============================================================

FILTER_ALPHA = 0.98

GYRO_DEADBAND = 0.20


# ============================================================
# MOTION / TRANSLATION DISPLAY
# ============================================================

# The whole hand model is shifted on screen by the estimated
# position.  Position is in meters and tiny, so we scale it
# into plot units.  Bigger MOTION_SCALE = hand moves more for
# the same physical movement.  Increase it if the hand barely
# moves; decrease it if the hand flies off screen.
MOTION_SCALE = 7.0

# Never let the hand leave the plotting box (+/- 3 units).
MOTION_CLAMP = 2.5


# ============================================================
# CALIBRATION
# ============================================================

# When the program starts (and whenever you press C) it spends
# a few seconds averaging the sensor while you hold it STILL.
#
#   gyro  -> the average is the "zero-rate bias".  Subtracting
#            it stops the orientation (especially yaw) from
#            slowly spinning on its own.
#
#   accel -> the average magnitude should be exactly 1 g.  We
#            use it to scale the accelerometer so gravity reads
#            1.000 g, which makes the gravity-removal in the
#            motion code much cleaner (less position drift).

CALIB_WARMUP = 1.5     # seconds to settle before averaging starts
CALIB_SECONDS = 3.0    # seconds of still data to average


# ============================================================
# MOTION SMOOTHING / DRIFT CONTROL
# ============================================================

# These are TIME based (seconds), so the feel is the same no
# matter how fast BLE delivers packets.  The old version leaked
# a fixed fraction PER PACKET, which over-damped at high packet
# rates and made movement (especially up/down) feel weak/laggy.

# Velocity leak: larger = motion coasts longer (more responsive
# but drifts more).  Smaller = more damping.
LIN_VEL_TAU = 0.7      # seconds

# Slow pull of the hand back to center so it can't wander off.
# Larger = holds a pose longer; smaller = snaps back faster.
POS_RECENTER_TAU = 6.0  # seconds

# How quickly the resting linear-acceleration offset is learned
# while the hand is still.  This offset is what kills the
# constant vertical bias that made UP/DOWN barely register.
LIN_BIAS_ALPHA = 0.03  # per still sample (0..1)


# ============================================================
# GUIDED CALIBRATION (press G)
#
# A step-by-step wizard.  First it measures a STILL rest pose
# (gyro bias, gravity), then it asks you to PUSH the device in
# each of the 6 directions, one at a time on a timer.  It reads
# the direction of each push and works out which sensor axis is
# up, which is left/right, which is forward/back - so every
# direction lands on the correct graph axis.
# ============================================================

WIZ_PREP = 2.0        # seconds "get ready" before each push
WIZ_CAP = 3.0         # seconds capture window per direction

WIZ_ONSET_G = 0.12    # g, motion must exceed this to count as a push
WIZ_ONSET_WIN = 0.35  # s, average the push over this long after it starts
WIZ_MIN_AXIS = 0.08   # g, reject the result if a push was too weak

# (key, on-screen label).  "still" must be first.
WIZARD_SEQUENCE = [
    ("still", "HOLD STILL"),
    ("up",    "PUSH UP"),
    ("down",  "PUSH DOWN"),
    ("left",  "PUSH LEFT"),
    ("right", "PUSH RIGHT"),
    ("fwd",   "PUSH FORWARD (away)"),
    ("back",  "PUSH BACK (toward you)"),
]


# Change signs if physical motion is opposite to graph

ACC_X_SIGN = +1
ACC_Y_SIGN = +1
ACC_Z_SIGN = +1

GYRO_X_SIGN = +1
GYRO_Y_SIGN = +1
GYRO_Z_SIGN = +1


# ============================================================
# ADS1299
# ============================================================

ADS_START = 13

ADS_CHANNELS = 8

ADS_BYTES_PER_CHANNEL = 3

ADS_SAMPLE_SIZE = 24

ADS_SAMPLES_PER_PACKET = 4

ADS_PAYLOAD_SIZE = (
    ADS_SAMPLE_SIZE *
    ADS_SAMPLES_PER_PACKET
)

# 13 + 96 = 109

assert ADS_START + ADS_PAYLOAD_SIZE == PACKET_SIZE


# ============================================================
# ADS1299 -> MICROVOLTS
# ============================================================

# Firmware:
# CONFIG3 internal reference
# CH1SET..CH8SET = 0x00
#
# PGA gain = 1

ADS_VREF = 4.5
ADS_GAIN = 1.0

ADS_UV_PER_COUNT = (
    ADS_VREF *
    1_000_000.0 /
    (
        ADS_GAIN *
        (2 ** 23)
    )
)

print()
print("============================================")
print(" PiEEG_XR")
print("============================================")
print(f"BLE packet       : {PACKET_SIZE} bytes")
print(f"ADS samples      : {ADS_SAMPLES_PER_PACKET}")
print(f"ADS channels     : {ADS_CHANNELS}")
print(f"ADS gain         : {ADS_GAIN}")
print(f"ADS VREF         : {ADS_VREF} V")
print(
    f"ADC conversion   : "
    f"{ADS_UV_PER_COUNT:.9f} uV/count"
)
print("============================================")
print()


# ============================================================
# GRAPH SETTINGS
# ============================================================

EMG_WINDOW = 800


# ============================================================
# SHARED DATA
# ============================================================

class SensorData:

    def __init__(self):

        self.lock = threading.Lock()

        # orientation

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.zero_roll = 0.0
        self.zero_pitch = 0.0
        self.zero_yaw = 0.0

        self.last_imu_time = None

        # accelerometer

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

        # gyro

        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0

        # motion (world-frame linear accel / velocity / position)

        self.lin = np.zeros(3)   # linear accel, gravity removed, m/s^2
        self.vel = np.zeros(3)   # m/s
        self.pos = np.zeros(3)   # m

        # resting linear-accel offset, learned while still.
        # subtracted from lin so a motionless hand reads ~0 on
        # every axis (this is what fixes weak up/down).
        self.lin_bias = np.zeros(3)

        # calibration

        self.gyro_bias = np.zeros(3)   # deg/s, subtracted from gyro
        self.acc_scale = 1.0           # multiplies accel so gravity = 1 g

        # axis alignment: rotates the raw sensor axes so that
        # whatever direction is physically UP becomes graph Z.
        # Identity until calibration measures the real mounting.
        self.R_align = np.eye(3)

        self.calibrating = False
        self.calibrated = False

        self.calib_acc = []
        self.calib_gyro = []

        self.calib_warmup_until = 0.0
        self.calib_end = 0.0

        # when a calibration finishes we auto-zero orientation
        self.zero_pending = False

        # timestamp calibration finished, for the on-screen flash
        self.calib_done_time = 0.0

        # guided calibration wizard state
        self.wizard_active = False
        self.wizard_idx = 0
        self.wizard_prep_until = 0.0
        self.wizard_cap_end = 0.0
        self.wizard_banner = ""
        self.wizard_peaks = {}

        self.wiz_still_acc = []
        self.wiz_still_gyro = []
        self.wiz_g_sensor = np.array([0.0, 0.0, 1.0])

        self.wiz_samples = []
        self.wiz_onset = False
        self.wiz_onset_time = 0.0

        # EMG buffers

        self.emg = [

            deque(maxlen=EMG_WINDOW)

            for _ in range(ADS_CHANNELS)
        ]

        self.sample_numbers = deque(
            maxlen=EMG_WINDOW
        )

        self.emg_counter = 0

        # BLE

        self.packet_counter = 0
        self.bad_packet_counter = 0

        self.connected = False


sensor = SensorData()


# ============================================================
# ADS1299 SIGNED 24 BIT
# ============================================================

def signed24(b0, b1, b2):

    """
    ADS1299 sends:

        MSB
        middle byte
        LSB

    Signed 24-bit two's complement.
    """

    value = (
        (int(b0) << 16)
        |
        (int(b1) << 8)
        |
        int(b2)
    )

    if value & 0x800000:

        value -= 0x1000000

    return value


# ============================================================
# RAW -> MICROVOLTS
# ============================================================

def adc_to_microvolts(raw):

    return (
        raw *
        ADS_VREF *
        1_000_000.0 /
        (
            ADS_GAIN *
            (2 ** 23)
        )
    )


# ============================================================
# DECODE ONE ADS1299 SAMPLE
# ============================================================

def decode_ads_sample(block):

    if len(block) != 24:

        return None

    values_uv = []

    for channel in range(8):

        offset = channel * 3

        raw = signed24(

            block[offset],

            block[offset + 1],

            block[offset + 2]
        )

        uv = adc_to_microvolts(raw)

        values_uv.append(uv)

    return values_uv


# ============================================================
# DECODE 4 ADS SAMPLES
# ============================================================

def decode_ads1299(packet):

    if len(packet) < PACKET_SIZE:

        return

    # Exactly bytes 13..108

    ads = packet[
        ADS_START:
        PACKET_SIZE
    ]

    if len(ads) != 96:

        return

    for sample_index in range(
        ADS_SAMPLES_PER_PACKET
    ):

        start = (
            sample_index *
            ADS_SAMPLE_SIZE
        )

        end = (
            start +
            ADS_SAMPLE_SIZE
        )

        block = ads[
            start:end
        ]

        channels_uv = decode_ads_sample(
            block
        )

        if channels_uv is None:

            continue

        with sensor.lock:

            sensor.emg_counter += 1

            sensor.sample_numbers.append(
                sensor.emg_counter
            )

            for channel in range(8):

                sensor.emg[
                    channel
                ].append(
                    channels_uv[
                        channel
                    ]
                )


# ============================================================
# IMU DECODE
# ============================================================

def decode_imu(packet):

    if len(packet) < 13:

        return None

    ax_raw, ay_raw, az_raw = struct.unpack_from(
        "<hhh",
        packet,
        ACC_OFFSET
    )

    gx_raw, gy_raw, gz_raw = struct.unpack_from(
        "<hhh",
        packet,
        GYRO_OFFSET
    )

    ax = (
        ax_raw *
        ACC_G_PER_LSB *
        ACC_X_SIGN
    )

    ay = (
        ay_raw *
        ACC_G_PER_LSB *
        ACC_Y_SIGN
    )

    az = (
        az_raw *
        ACC_G_PER_LSB *
        ACC_Z_SIGN
    )

    gx = (
        gx_raw *
        GYRO_DPS_PER_LSB *
        GYRO_X_SIGN
    )

    gy = (
        gy_raw *
        GYRO_DPS_PER_LSB *
        GYRO_Y_SIGN
    )

    gz = (
        gz_raw *
        GYRO_DPS_PER_LSB *
        GYRO_Z_SIGN
    )

    return (
        ax,
        ay,
        az,
        gx,
        gy,
        gz
    )


# ============================================================
# AXIS ALIGNMENT
#
# Given the gravity direction measured in the sensor's own
# axes (while still), build the smallest rotation that turns
# that direction into +Z.  Multiplying every accel/gyro
# reading by this rotation makes "physically up" always come
# out as graph Z, no matter how the chip is mounted.
# ============================================================

def alignment_rotation(gravity_sensor):

    g = np.asarray(gravity_sensor, dtype=float)

    norm = np.linalg.norm(g)

    if norm < 1e-6:
        return np.eye(3)

    g = g / norm

    target = np.array([0.0, 0.0, 1.0])

    v = np.cross(g, target)
    c = float(np.dot(g, target))
    s = float(np.linalg.norm(v))

    # Already pointing up.
    if s < 1e-8:

        if c > 0:
            return np.eye(3)

        # Pointing straight down: flip 180 deg about X.
        return np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ])

    # Rodrigues' rotation formula.
    vx = np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])

    R = (
        np.eye(3)
        + vx
        + vx @ vx * ((1.0 - c) / (s * s))
    )

    return R


# ============================================================
# GYRO NOISE DEAD BAND
# ============================================================

def apply_deadband(value):

    if abs(value) < GYRO_DEADBAND:

        return 0.0

    return value


# ============================================================
# CALIBRATION HELPERS
#
# Both assume sensor.lock is ALREADY held by the caller.
# ============================================================

def begin_calibration(now):

    sensor.calibrating = True

    sensor.calib_acc = []
    sensor.calib_gyro = []

    # First warm up (user gets ready / device settles),
    # then average for CALIB_SECONDS.
    sensor.calib_warmup_until = now + CALIB_WARMUP
    sensor.calib_end = sensor.calib_warmup_until + CALIB_SECONDS

    print()
    print("--------------------------------------------")
    print(" CALIBRATION")
    print(" Hold the device COMPLETELY STILL and flat.")
    print(f" Averaging for {CALIB_SECONDS:.0f} s "
          f"(after a {CALIB_WARMUP:.1f} s pause).")
    print("--------------------------------------------")


def finish_calibration():

    # Need a reasonable number of samples to trust the average.
    if len(sensor.calib_gyro) < 5:

        print("CALIBRATION: too few samples, retrying...")

        now = time.perf_counter()
        sensor.calib_warmup_until = now + CALIB_WARMUP
        sensor.calib_end = sensor.calib_warmup_until + CALIB_SECONDS
        sensor.calib_acc = []
        sensor.calib_gyro = []
        return

    gyro_arr = np.array(sensor.calib_gyro)
    acc_arr = np.array(sensor.calib_acc)

    # Gyro bias = average reading while still.
    sensor.gyro_bias = gyro_arr.mean(axis=0)

    # Accel scale = make the resting gravity vector read 1 g.
    mean_acc = acc_arr.mean(axis=0)
    mag = float(np.linalg.norm(mean_acc))

    if mag > 0.1:
        sensor.acc_scale = 1.0 / mag
    else:
        sensor.acc_scale = 1.0

    # ----------------------------------------------------
    # AXIS ALIGNMENT
    # The resting accel points along "up" in the sensor's
    # own axes.  Build the rotation that turns that into
    # graph Z so that physically-up = up on the screen.
    # ----------------------------------------------------
    sensor.R_align = alignment_rotation(mean_acc)

    # Report which sensor axis was pointing up, in plain terms.
    g_unit = mean_acc / mag if mag > 1e-6 else mean_acc
    axis_names = ["X", "Y", "Z"]
    dominant = int(np.argmax(np.abs(g_unit)))
    sign = "+" if g_unit[dominant] > 0 else "-"

    sensor.calibrating = False
    sensor.calibrated = True

    # Start clean: re-init orientation, wipe motion, and make
    # this rest pose the on-screen zero.
    sensor.last_imu_time = None
    sensor.vel[:] = 0.0
    sensor.pos[:] = 0.0
    sensor.lin_bias[:] = 0.0
    sensor.zero_pending = True

    sensor.calib_done_time = time.perf_counter()

    print("CALIBRATION done.")
    print(f"  gyro bias   : "
          f"{sensor.gyro_bias[0]:+.3f}  "
          f"{sensor.gyro_bias[1]:+.3f}  "
          f"{sensor.gyro_bias[2]:+.3f}  deg/s")
    print(f"  accel scale : {sensor.acc_scale:.4f}  "
          f"(resting gravity measured {mag:.3f} g)")
    print(f"  up axis     : sensor {sign}{axis_names[dominant]}  "
          f"-> remapped to graph Z")
    print()


# ============================================================
# GUIDED CALIBRATION WIZARD
#
# All of these assume sensor.lock is already held.
# ============================================================

def begin_wizard(now):

    sensor.wizard_active = True
    sensor.calibrating = False
    sensor.calibrated = False

    sensor.wizard_idx = 0
    sensor.wizard_peaks = {}

    sensor.wiz_still_acc = []
    sensor.wiz_still_gyro = []

    sensor.wiz_samples = []
    sensor.wiz_onset = False
    sensor.wiz_onset_time = 0.0

    sensor.wizard_prep_until = now + WIZ_PREP
    sensor.wizard_cap_end = sensor.wizard_prep_until + WIZ_CAP

    print()
    print("--------------------------------------------")
    print(" GUIDED CALIBRATION")
    print(" Follow the on-screen prompts.")
    print(" Give ONE clear push per direction, then stop.")
    print("--------------------------------------------")


def wizard_advance(now):

    sensor.wizard_idx += 1

    if sensor.wizard_idx >= len(WIZARD_SEQUENCE):

        finish_wizard()
        return

    sensor.wizard_prep_until = now + WIZ_PREP
    sensor.wizard_cap_end = sensor.wizard_prep_until + WIZ_CAP

    sensor.wiz_samples = []
    sensor.wiz_onset = False
    sensor.wiz_onset_time = 0.0


def wizard_finish_still():

    if len(sensor.wiz_still_gyro) >= 5:

        gyro_arr = np.array(sensor.wiz_still_gyro)
        acc_arr = np.array(sensor.wiz_still_acc)

        sensor.gyro_bias = gyro_arr.mean(axis=0)

        mean_acc = acc_arr.mean(axis=0)
        mag = float(np.linalg.norm(mean_acc))

        sensor.acc_scale = 1.0 / mag if mag > 0.1 else 1.0

        # gravity vector in scaled units (magnitude ~1 g)
        sensor.wiz_g_sensor = mean_acc * sensor.acc_scale


def build_alignment_from_peaks(peaks):

    needed = ["up", "down", "left", "right", "fwd", "back"]

    for k in needed:
        if k not in peaks:
            return None

    vertical = peaks["up"] - peaks["down"]
    lateral = peaks["right"] - peaks["left"]
    forward = peaks["fwd"] - peaks["back"]

    # Reject if any push was too weak to trust.
    if (np.linalg.norm(vertical) < WIZ_MIN_AXIS or
            np.linalg.norm(lateral) < WIZ_MIN_AXIS or
            np.linalg.norm(forward) < WIZ_MIN_AXIS):
        return None

    def unit(v):
        n = np.linalg.norm(v)
        return v / n if n > 1e-9 else None

    lat = unit(lateral)
    fwd = unit(forward)
    ver = unit(vertical)

    if lat is None or fwd is None or ver is None:
        return None

    # Columns map graph axes -> sensor directions.
    A = np.column_stack([lat, fwd, ver])

    # Nearest true rotation (fixes small non-orthogonality from
    # imperfect hand movements) via SVD.
    U, _, Vt = np.linalg.svd(A)
    Rs = U @ Vt

    if np.linalg.det(Rs) < 0:
        U[:, -1] *= -1.0
        Rs = U @ Vt

    # sensor -> graph is the transpose.
    return Rs.T


def finish_wizard():

    R = build_alignment_from_peaks(sensor.wizard_peaks)

    if R is None:

        # Movements too weak: fall back to gravity-only alignment.
        R = alignment_rotation(sensor.wiz_g_sensor)
        print("GUIDED CALIBRATION: pushes too weak, "
              "used gravity (up/down) only.")

    else:

        print("GUIDED CALIBRATION done. All 6 directions mapped.")

    sensor.R_align = R

    sensor.wizard_active = False
    sensor.calibrated = True
    sensor.calibrating = False

    sensor.last_imu_time = None
    sensor.vel[:] = 0.0
    sensor.pos[:] = 0.0
    sensor.lin_bias[:] = 0.0
    sensor.zero_pending = True

    sensor.calib_done_time = time.perf_counter()
    sensor.wizard_banner = ""

    print()


def handle_wizard(now, ax, ay, az, gx, gy, gz):

    # keep the on-screen numbers live
    sensor.ax, sensor.ay, sensor.az = ax, ay, az
    sensor.gx, sensor.gy, sensor.gz = gx, gy, gz

    key, label = WIZARD_SEQUENCE[sensor.wizard_idx]

    # ---- prep / "get ready" phase ----
    if now < sensor.wizard_prep_until:

        remaining = sensor.wizard_prep_until - now
        sensor.wizard_banner = f"GET READY:  {label}   {remaining:0.0f}"
        return

    remaining = max(0.0, sensor.wizard_cap_end - now)

    # ---- STILL step: average bias / gravity ----
    if key == "still":

        sensor.wizard_banner = f"{label}   {remaining:0.1f}s"

        sensor.wiz_still_acc.append((ax, ay, az))
        sensor.wiz_still_gyro.append((gx, gy, gz))

        if now >= sensor.wizard_cap_end:
            wizard_finish_still()
            wizard_advance(now)

        return

    # ---- MOTION step: capture the push direction ----
    acc = np.array([ax, ay, az]) * sensor.acc_scale

    # gravity removed, in g (orientation ~ unchanged during a push)
    lin = acc - sensor.wiz_g_sensor
    mag = float(np.linalg.norm(lin))

    if not sensor.wiz_onset:

        # wait for the push to start
        if mag > WIZ_ONSET_G:
            sensor.wiz_onset = True
            sensor.wiz_onset_time = now
            sensor.wiz_samples = [lin]

        sensor.wizard_banner = f"{label}  NOW!   {remaining:0.1f}s"

    else:

        # average only the first moments of the push, so that
        # the "stop" deceleration afterwards can't flip the sign
        if now - sensor.wiz_onset_time <= WIZ_ONSET_WIN:
            sensor.wiz_samples.append(lin)

        sensor.wizard_banner = f"{label}  got it   {remaining:0.1f}s"

    if now >= sensor.wizard_cap_end:

        if sensor.wiz_samples:
            peak = np.mean(np.array(sensor.wiz_samples), axis=0)
        else:
            peak = np.zeros(3)

        sensor.wizard_peaks[key] = peak
        wizard_advance(now)


# ============================================================
# UPDATE ORIENTATION
# ============================================================

def update_orientation(
    ax,
    ay,
    az,
    gx,
    gy,
    gz
):

    now = time.perf_counter()

    with sensor.lock:

        # ----------------------------------------------------
        # GUIDED WIZARD (press G) takes priority when active.
        # ----------------------------------------------------

        if sensor.wizard_active:

            handle_wizard(now, ax, ay, az, gx, gy, gz)
            return

        # ----------------------------------------------------
        # CALIBRATION
        #
        # Auto-start once, the first time real data arrives.
        # While calibrating we only collect still samples and
        # do NOT move the hand.
        # ----------------------------------------------------

        if (not sensor.calibrated) and (not sensor.calibrating):

            begin_calibration(now)

        if sensor.calibrating:

            # keep the on-screen numbers live during calibration
            sensor.ax, sensor.ay, sensor.az = ax, ay, az
            sensor.gx, sensor.gy, sensor.gz = gx, gy, gz

            # after the warm-up pause, start averaging
            if now >= sensor.calib_warmup_until:

                sensor.calib_acc.append((ax, ay, az))
                sensor.calib_gyro.append((gx, gy, gz))

            if now >= sensor.calib_end:

                finish_calibration()

            return

        # ----------------------------------------------------
        # APPLY CALIBRATION
        #   - subtract gyro bias   (in raw sensor axes)
        #   - scale accel so gravity = 1 g
        #   - remap axes so physical "up" becomes graph Z
        # ----------------------------------------------------

        gx = gx - sensor.gyro_bias[0]
        gy = gy - sensor.gyro_bias[1]
        gz = gz - sensor.gyro_bias[2]

        ax = ax * sensor.acc_scale
        ay = ay * sensor.acc_scale
        az = az * sensor.acc_scale

        # Axis alignment: rotate both accel and gyro into the
        # graph frame.  A rotation transforms both the same way.
        acc_vec = sensor.R_align @ np.array([ax, ay, az])
        gyro_vec = sensor.R_align @ np.array([gx, gy, gz])

        ax, ay, az = acc_vec[0], acc_vec[1], acc_vec[2]
        gx, gy, gz = gyro_vec[0], gyro_vec[1], gyro_vec[2]

        # gyro noise dead-band (after bias removal + alignment)
        gx = apply_deadband(gx)
        gy = apply_deadband(gy)
        gz = apply_deadband(gz)

        sensor.ax = ax
        sensor.ay = ay
        sensor.az = az

        sensor.gx = gx
        sensor.gy = gy
        sensor.gz = gz

        # ----------------------------------------------------
        # First IMU packet
        # ----------------------------------------------------

        if sensor.last_imu_time is None:

            sensor.last_imu_time = now

            sensor.roll = math.degrees(
                math.atan2(
                    ay,
                    az
                )
            )

            sensor.pitch = math.degrees(
                math.atan2(
                    -ax,
                    math.sqrt(
                        ay * ay +
                        az * az
                    )
                )
            )

            sensor.yaw = 0.0

            # if a calibration just finished, make this rest
            # pose the on-screen zero automatically
            if sensor.zero_pending:

                sensor.zero_roll = sensor.roll
                sensor.zero_pitch = sensor.pitch
                sensor.zero_yaw = sensor.yaw

                sensor.zero_pending = False

            return

        # ----------------------------------------------------
        # dt
        # ----------------------------------------------------

        dt = (
            now -
            sensor.last_imu_time
        )

        sensor.last_imu_time = now

        if dt <= 0:

            return

        # Clamp instead of dropping late packets.  Dropping made
        # the motion stutter whenever BLE delivered a burst of
        # packets; clamping keeps the update smooth.
        if dt > 0.10:

            dt = 0.10

        # ----------------------------------------------------
        # Gyro angle
        # ----------------------------------------------------

        gyro_roll = (
            sensor.roll +
            gx * dt
        )

        gyro_pitch = (
            sensor.pitch +
            gy * dt
        )

        gyro_yaw = (
            sensor.yaw +
            gz * dt
        )

        # ----------------------------------------------------
        # Accelerometer angle
        # ----------------------------------------------------

        acc_roll = math.degrees(
            math.atan2(
                ay,
                az
            )
        )

        acc_pitch = math.degrees(
            math.atan2(
                -ax,
                math.sqrt(
                    ay * ay +
                    az * az
                )
            )
        )

        magnitude = math.sqrt(
            ax * ax +
            ay * ay +
            az * az
        )

        # ----------------------------------------------------
        # Complementary filter
        # ----------------------------------------------------

        if 0.75 < magnitude < 1.25:

            sensor.roll = (
                FILTER_ALPHA *
                gyro_roll
                +
                (1.0 - FILTER_ALPHA) *
                acc_roll
            )

            sensor.pitch = (
                FILTER_ALPHA *
                gyro_pitch
                +
                (1.0 - FILTER_ALPHA) *
                acc_pitch
            )

        else:

            sensor.roll = gyro_roll
            sensor.pitch = gyro_pitch

        # LSM6DS3 does not have magnetometer,
        # therefore yaw comes from gyro only.

        sensor.yaw = gyro_yaw

        # ----------------------------------------------------
        # LINEAR ACCELERATION (gravity removed, world frame)
        #
        # The accelerometer always reads ~1 g of gravity mixed
        # with real motion.  Rotate the reading into the world
        # frame using the current orientation, then subtract
        # gravity ([0, 0, +1] g because Z is up in the plot).
        # What remains is the acceleration caused by MOVING
        # the hand, in m/s^2.
        # ----------------------------------------------------

        R = rotation_matrix(
            sensor.roll,
            sensor.pitch,
            sensor.yaw
        )

        acc_world = R @ np.array([ax, ay, az])

        lin = (
            acc_world -
            np.array([0.0, 0.0, 1.0])
        ) * 9.81

        # ----------------------------------------------------
        # POSITION ESTIMATE with ZUPT + adaptive bias
        #
        # Two problems the old code had:
        #   1. It subtracted EXACTLY 1 g, so any residual offset
        #      (very common on Z) leaked in and made up/down
        #      feel dead or drift.  Fix: while the hand is still,
        #      learn the leftover offset (lin_bias) and subtract
        #      it, so a motionless hand reads ~0 on every axis.
        #   2. The leak was per-packet, so at high BLE rates it
        #      crushed the motion.  Fix: time-based leaks.
        # ----------------------------------------------------

        acc_mag = math.sqrt(
            ax * ax +
            ay * ay +
            az * az
        )

        gyro_mag = math.sqrt(
            gx * gx +
            gy * gy +
            gz * gz
        )

        is_still = (
            abs(acc_mag - 1.0) < 0.05
            and
            gyro_mag < 5.0
        )

        # Learn the resting offset only while genuinely still,
        # so we never subtract real movement.
        if is_still:

            sensor.lin_bias = (
                (1.0 - LIN_BIAS_ALPHA) * sensor.lin_bias
                +
                LIN_BIAS_ALPHA * lin
            )

        # Bias-corrected linear acceleration (what we integrate
        # and what the direction label reads).
        lin = lin - sensor.lin_bias

        sensor.lin = lin

        if is_still:

            sensor.vel[:] = 0.0

        else:

            sensor.vel += lin * dt

            # time-based velocity leak (coast then settle)
            sensor.vel *= math.exp(-dt / LIN_VEL_TAU)

            sensor.pos += sensor.vel * dt

        # time-based slow pull back to center
        sensor.pos *= math.exp(-dt / POS_RECENTER_TAU)


# ============================================================
# PROCESS BLE NOTIFICATION
# ============================================================

def process_packet(packet):

    packet = bytes(packet)

    # --------------------------------------------------------
    # Verify packet length
    # --------------------------------------------------------

    if len(packet) < PACKET_SIZE:

        with sensor.lock:

            sensor.bad_packet_counter += 1

        print(
            f"Bad packet: "
            f"{len(packet)} bytes "
            f"(expected {PACKET_SIZE})"
        )

        return

    # If BLE somehow gives more data, only process
    # first 109 bytes.

    packet = packet[:PACKET_SIZE]

    with sensor.lock:

        sensor.packet_counter += 1

    # --------------------------------------------------------
    # IMU
    # --------------------------------------------------------

    imu = decode_imu(
        packet
    )

    if imu is not None:

        update_orientation(
            *imu
        )

    # --------------------------------------------------------
    # ADS1299
    # --------------------------------------------------------

    decode_ads1299(
        packet
    )


# ============================================================
# BLE
# ============================================================

async def ble_main(
    address,
    stop_event
):

    # --------------------------------------------------------
    # Find device
    # --------------------------------------------------------

    if address:

        print(
            "Searching address:",
            address
        )

        device = await BleakScanner.find_device_by_address(
            address,
            timeout=10.0
        )

    else:

        print(
            "Searching for",
            DEVICE_NAME
        )

        device = await BleakScanner.find_device_by_filter(

            lambda d, ad:

            (
                ad.local_name ==
                DEVICE_NAME
            )

            or

            (
                d.name ==
                DEVICE_NAME
            ),

            timeout=10.0
        )

    if device is None:

        print(
            "PiEEG_XR not found"
        )

        stop_event.set()

        return

    print(
        "Found:",
        device.address
    )

    print(
        "Connecting..."
    )

    # --------------------------------------------------------
    # Callback
    # --------------------------------------------------------

    def notification_handler(
        sender,
        packet
    ):

        process_packet(
            packet
        )

    # --------------------------------------------------------
    # Connect
    # --------------------------------------------------------

    async with BleakClient(
        device
    ) as client:

        with sensor.lock:

            sensor.connected = True

        print(
            "BLE connected"
        )

        await client.start_notify(
            NOTIFY_UUID,
            notification_handler
        )

        try:

            while not stop_event.is_set():

                await asyncio.sleep(
                    0.1
                )

        finally:

            with sensor.lock:

                sensor.connected = False

            try:

                await client.stop_notify(
                    NOTIFY_UUID
                )

            except Exception:

                pass


# ============================================================
# BLE THREAD
# ============================================================

def start_ble(
    address,
    stop_event
):

    def worker():

        loop = asyncio.new_event_loop()

        asyncio.set_event_loop(
            loop
        )

        try:

            loop.run_until_complete(
                ble_main(
                    address,
                    stop_event
                )
            )

        except Exception as error:

            print(
                "BLE ERROR:",
                error
            )

            stop_event.set()

        finally:

            loop.close()

    thread = threading.Thread(
        target=worker,
        daemon=True
    )

    thread.start()


# ============================================================
# ROTATION MATRIX
# ============================================================

def rotation_matrix(
    roll_deg,
    pitch_deg,
    yaw_deg
):

    roll = math.radians(
        roll_deg
    )

    pitch = math.radians(
        pitch_deg
    )

    yaw = math.radians(
        yaw_deg
    )

    Rx = np.array([
        [1, 0, 0],

        [
            0,
            math.cos(roll),
            -math.sin(roll)
        ],

        [
            0,
            math.sin(roll),
            math.cos(roll)
        ]
    ])

    Ry = np.array([

        [
            math.cos(pitch),
            0,
            math.sin(pitch)
        ],

        [0, 1, 0],

        [
            -math.sin(pitch),
            0,
            math.cos(pitch)
        ]
    ])

    Rz = np.array([

        [
            math.cos(yaw),
            -math.sin(yaw),
            0
        ],

        [
            math.sin(yaw),
            math.cos(yaw),
            0
        ],

        [0, 0, 1]
    ])

    return (
        Rz @
        Ry @
        Rx
    )


# ============================================================
# OBJ HAND MODEL
# ============================================================

# By default the script looks for this OBJ beside the Python file.
DEFAULT_HAND_OBJ = "hand_fast.obj"

# Real-time safety limit. Any OBJ (triangles/quads/polygons) is triangulated,
# then evenly downsampled to this many displayed triangles.
# BLE acquisition stays independent; this only limits GUI rendering cost.
HAND_MAX_FACES = 800

# The OBJ is much larger than the plotting coordinate system.  It is centered
# and scaled so its longest dimension is this size in graph units.
HAND_DISPLAY_LENGTH = 3.6


def load_obj_mesh(filename, max_faces=HAND_MAX_FACES):
    """Load vertices and polygon faces from a Wavefront OBJ file.

    Only geometry is needed for the IMU display, so materials/textures/normals
    are intentionally ignored.  OBJ face references such as 12/4/9 are
    supported, as are negative vertex indices.
    """

    vertices = []
    faces = []

    with open(filename, "r", encoding="utf-8", errors="ignore") as obj_file:
        for line in obj_file:
            if line.startswith("v "):
                parts = line.split()
                if len(parts) >= 4:
                    vertices.append([
                        float(parts[1]),
                        float(parts[2]),
                        float(parts[3]),
                    ])

            elif line.startswith("f "):
                items = line.split()[1:]
                face = []

                for item in items:
                    vertex_text = item.split("/")[0]
                    if not vertex_text:
                        continue

                    index = int(vertex_text)

                    if index > 0:
                        index -= 1
                    else:
                        index = len(vertices) + index

                    face.append(index)

                # OBJ files may contain triangles, quads, or larger polygons.
                # Convert every polygon to triangles using a triangle fan so
                # faces_index is always a rectangular NumPy array.
                if len(face) >= 3:
                    for i in range(1, len(face) - 1):
                        faces.append([
                            face[0],
                            face[i],
                            face[i + 1],
                        ])

    vertices = np.asarray(vertices, dtype=float)

    if vertices.size == 0 or not faces:
        raise ValueError(f"No usable mesh geometry found in {filename}")

    # Center the model around the origin.
    mesh_min = vertices.min(axis=0)
    mesh_max = vertices.max(axis=0)
    center = (mesh_min + mesh_max) / 2.0
    vertices = vertices - center

    # Scale longest dimension to HAND_DISPLAY_LENGTH.
    extent = mesh_max - mesh_min
    longest = float(np.max(extent))
    if longest <= 0.0:
        raise ValueError(f"Invalid mesh dimensions in {filename}")

    vertices *= HAND_DISPLAY_LENGTH / longest

    # Keep the long hand direction aligned with local X, matching the old box.
    # The supplied model is already longest along X, so no fixed rotation is
    # required here.  If another model needs alignment, apply it here once.

    # Evenly select faces when the mesh is very dense.  This is a display-only
    # optimization; the original OBJ file is not modified.
    if max_faces is not None and len(faces) > max_faces:
        selection = np.linspace(
            0,
            len(faces) - 1,
            max_faces,
            dtype=int,
        )
        faces = [faces[i] for i in selection]

    return vertices, faces


# ============================================================
# MAIN
# ============================================================

def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--address",
        default=None
    )

    parser.add_argument(
        "--hand",
        default=None,
        help=(
            "Path to Wavefront OBJ hand model. "
            f"Default: {DEFAULT_HAND_OBJ} beside this script"
        )
    )

    args = parser.parse_args()

    stop_event = threading.Event()

    start_ble(
        args.address,
        stop_event
    )

    # ========================================================
    # FIGURE
    # ========================================================

    fig = plt.figure(
        figsize=(16, 9)
    )

    fig.suptitle(
        "PiEEG_XR - IMU + 8 Channel EMG"
    )


    # ========================================================
    # 3D VIEW
    # ========================================================

    ax3d = fig.add_axes(
        [
            0.02,
            0.08,
            0.48,
            0.82
        ],
        projection="3d"
    )


    # ========================================================
    # HAND OBJ MODEL
    # ========================================================

    if args.hand:
        hand_path = Path(args.hand).expanduser().resolve()
    else:
        hand_path = (
            Path(__file__).resolve().parent /
            DEFAULT_HAND_OBJ
        )

    if not hand_path.exists():
        raise FileNotFoundError(
            "Hand OBJ not found: "
            f"{hand_path}\n\n"
            "Put hand_fast.obj in the same folder as this script, "
            "or run with --hand /path/to/model.obj"
        )

    print("Loading hand model:", hand_path)

    vertices, faces_index = load_obj_mesh(
        hand_path,
        max_faces=HAND_MAX_FACES,
    )

    print(
        f"Hand mesh: {len(vertices)} vertices, "
        f"{len(faces_index)} displayed triangles"
    )

    # Convert triangular face indices to one NumPy array.  This avoids
    # rebuilding thousands of Python lists on every animation frame.
    faces_index = np.asarray(faces_index, dtype=np.int32)

    faces = vertices[faces_index]

    body = Poly3DCollection(
        faces,
        alpha=0.90,
        edgecolor="none",
        linewidth=0.0
    )

    ax3d.add_collection3d(body)


    # ========================================================
    # HAND X Y Z AXES
    # ========================================================

    body_x, = ax3d.plot(
        [],
        [],
        [],
        linewidth=3
    )

    body_y, = ax3d.plot(
        [],
        [],
        [],
        linewidth=3
    )

    body_z, = ax3d.plot(
        [],
        [],
        [],
        linewidth=3
    )


    # ========================================================
    # MOTION MARKER + DIRECTION LABEL
    # ========================================================

    # Magenta dot = estimated translation of the hand (ZUPT).
    hand_marker, = ax3d.plot(
        [0],
        [0],
        [0],
        marker="o",
        markersize=12,
        color="magenta"
    )

    # Big text showing detected motion direction, e.g. "LEFT".
    motion_text = ax3d.text2D(
        0.02,
        0.02,
        "",
        transform=ax3d.transAxes,
        fontsize=14,
        color="magenta"
    )


    # ========================================================
    # WORLD AXES
    # ========================================================

    ax3d.plot(
        [0, 2.5],
        [0, 0],
        [0, 0],
        linestyle="--"
    )

    ax3d.plot(
        [0, 0],
        [0, 2.5],
        [0, 0],
        linestyle="--"
    )

    ax3d.plot(
        [0, 0],
        [0, 0],
        [0, 2.5],
        linestyle="--"
    )


    ax3d.text(
        2.6,
        0,
        0,
        "X"
    )

    ax3d.text(
        0,
        2.6,
        0,
        "Y"
    )

    ax3d.text(
        0,
        0,
        2.6,
        "Z"
    )


    ax3d.set_xlim(
        -3,
        3
    )

    ax3d.set_ylim(
        -3,
        3
    )

    ax3d.set_zlim(
        -3,
        3
    )

    ax3d.set_box_aspect(
        (1, 1, 1)
    )

    ax3d.set_xlabel(
        "World X"
    )

    ax3d.set_ylabel(
        "World Y"
    )

    ax3d.set_zlabel(
        "World Z"
    )

    ax3d.view_init(
        elev=25,
        azim=-60
    )


    # ========================================================
    # 8 EMG GRAPHS
    # ========================================================

    emg_axes = []

    emg_lines = []

    graph_left = 0.57

    graph_width = 0.40

    graph_height = 0.084

    graph_spacing = 0.103

    first_bottom = 0.86


    for channel in range(8):

        bottom = (
            first_bottom -
            channel *
            graph_spacing
        )

        ax = fig.add_axes(
            [
                graph_left,
                bottom,
                graph_width,
                graph_height
            ]
        )

        line, = ax.plot(
            [],
            [],
            linewidth=0.9
        )

        ax.set_ylabel(

            f"CH{channel + 1}\nµV",

            rotation=0,

            labelpad=28
        )

        ax.grid(
            True,
            alpha=0.25
        )

        if channel != 7:

            ax.tick_params(
                labelbottom=False
            )

        else:

            ax.set_xlabel(
                "Sample"
            )

        emg_axes.append(
            ax
        )

        emg_lines.append(
            line
        )


    # ========================================================
    # KEYS:  R = zero orientation   C = calibrate
    # ========================================================

    def key_press(event):

        if event.key:

            if event.key.lower() == "r":

                with sensor.lock:

                    sensor.zero_roll = (
                        sensor.roll
                    )

                    sensor.zero_pitch = (
                        sensor.pitch
                    )

                    sensor.zero_yaw = (
                        sensor.yaw
                    )

                    # also recenter the translated hand
                    sensor.vel[:] = 0.0
                    sensor.pos[:] = 0.0

                print(
                    "Orientation ZERO"
                )

            elif event.key.lower() == "c":

                with sensor.lock:

                    begin_calibration(
                        time.perf_counter()
                    )

            elif event.key.lower() == "g":

                with sensor.lock:

                    begin_wizard(
                        time.perf_counter()
                    )


    fig.canvas.mpl_connect(
        "key_press_event",
        key_press
    )


    # ========================================================
    # ANIMATION
    # ========================================================

    def animate(frame):

        with sensor.lock:

            roll = (
                sensor.roll -
                sensor.zero_roll
            )

            pitch = (
                sensor.pitch -
                sensor.zero_pitch
            )

            yaw = (
                sensor.yaw -
                sensor.zero_yaw
            )

            ax_value = sensor.ax
            ay_value = sensor.ay
            az_value = sensor.az

            gx_value = sensor.gx
            gy_value = sensor.gy
            gz_value = sensor.gz

            pos = sensor.pos.copy()
            lin = sensor.lin.copy()

            packets = (
                sensor.packet_counter
            )

            bad_packets = (
                sensor.bad_packet_counter
            )

            connected = (
                sensor.connected
            )

            calibrating = (
                sensor.calibrating
            )

            calibrated = (
                sensor.calibrated
            )

            calib_warmup_until = sensor.calib_warmup_until
            calib_end = sensor.calib_end
            calib_samples = len(sensor.calib_gyro)
            calib_done_time = sensor.calib_done_time

            wizard_active = sensor.wizard_active
            wizard_banner = sensor.wizard_banner

            samples = list(
                sensor.sample_numbers
            )

            emg_data = [

                list(channel)

                for channel
                in sensor.emg
            ]


        # ====================================================
        # 3D ORIENTATION
        # ====================================================

        R = rotation_matrix(
            roll,
            pitch,
            yaw
        )

        # ----------------------------------------------------
        # TRANSLATION OFFSET
        #
        # Convert the estimated position (meters) into plot
        # units and clamp it inside the box.  This is the
        # vector that moves the whole hand up/down/left/right
        # on screen when the device moves.
        # ----------------------------------------------------

        disp = np.clip(
            pos * MOTION_SCALE,
            -MOTION_CLAMP,
            MOTION_CLAMP
        )

        rotated = (
            vertices @
            R.T
        )

        # NumPy advanced indexing is much faster than a Python face loop.
        # Adding disp (shape (3,)) shifts every vertex by the same offset.
        rotated_faces = rotated[faces_index] + disp

        body.set_verts(rotated_faces)


        # ====================================================
        # HAND AXES
        # ====================================================

        axis_length = 2.2

        local_x = np.array([
            axis_length,
            0,
            0
        ])

        local_y = np.array([
            0,
            axis_length,
            0
        ])

        local_z = np.array([
            0,
            0,
            axis_length
        ])


        rx = R @ local_x

        ry = R @ local_y

        rz = R @ local_z


        # Each axis now starts at the (shifted) hand origin
        # "disp" and points along the rotated direction.

        body_x.set_data(
            [disp[0], disp[0] + rx[0]],
            [disp[1], disp[1] + rx[1]]
        )

        body_x.set_3d_properties(
            [disp[2], disp[2] + rx[2]]
        )


        body_y.set_data(
            [disp[0], disp[0] + ry[0]],
            [disp[1], disp[1] + ry[1]]
        )

        body_y.set_3d_properties(
            [disp[2], disp[2] + ry[2]]
        )


        body_z.set_data(
            [disp[0], disp[0] + rz[0]],
            [disp[1], disp[1] + rz[1]]
        )

        body_z.set_3d_properties(
            [disp[2], disp[2] + rz[2]]
        )


        # ====================================================
        # MOTION MARKER (sits at the hand origin)
        # ====================================================

        hand_marker.set_data(
            [disp[0]],
            [disp[1]]
        )

        hand_marker.set_3d_properties(
            [disp[2]]
        )


        # ====================================================
        # DIRECTION LABEL (left / right / up / down ...)
        #
        # This is the RELIABLE part.  It looks at the world-
        # frame linear acceleration and reports the dominant
        # direction of motion.  Tune "threshold" and, if the
        # labels are wrong for how the sensor sits on the
        # hand, swap the axes or flip the ACC_*_SIGN values.
        # ====================================================

        threshold = 2.0  # m/s^2

        directions = []

        if not calibrating and not wizard_active:

            if abs(lin[0]) > threshold:
                directions.append(
                    "RIGHT" if lin[0] > 0 else "LEFT"
                )

            if abs(lin[1]) > threshold:
                directions.append(
                    "BACK" if lin[1] > 0 else "FWD"
                )

            if abs(lin[2]) > threshold:
                directions.append(
                    "UP" if lin[2] > 0 else "DOWN"
                )

        # The big magenta banner is shared: during calibration
        # it shows a countdown, just after it shows CALIBRATED,
        # the rest of the time it shows the motion direction.

        now_t = time.perf_counter()

        if wizard_active:

            banner = wizard_banner

        elif calibrating:

            if now_t < calib_warmup_until:

                banner = "GET READY - HOLD STILL"

            else:

                remaining = max(0.0, calib_end - now_t)

                banner = (
                    f"CALIBRATING  {remaining:0.1f}s   "
                    f"(samples {calib_samples})"
                )

        elif (now_t - calib_done_time) < 3.0:

            banner = "CALIBRATED  OK"

        else:

            banner = "   ".join(directions)

        motion_text.set_text(banner)


        # ====================================================
        # 3D STATUS
        # ====================================================

        if connected:

            status = "BLE CONNECTED"

        else:

            status = "WAITING FOR BLE"


        if wizard_active:

            calib_line = ">>> GUIDED CALIBRATION <<<"

        elif calibrating:

            calib_line = ">>> CALIBRATING - HOLD STILL <<<"

        elif calibrated:

            calib_line = "Calibrated"

        else:

            calib_line = "Not calibrated"


        ax3d.set_title(

            f"{status}\n"
            f"{calib_line}\n\n"

            f"Roll  : {roll:7.2f}°\n"
            f"Pitch : {pitch:7.2f}°\n"
            f"Yaw   : {yaw:7.2f}°\n\n"

            f"ACC\n"
            f"X {ax_value:+.3f} g\n"
            f"Y {ay_value:+.3f} g\n"
            f"Z {az_value:+.3f} g\n\n"

            f"GYRO\n"
            f"X {gx_value:+.2f} °/s\n"
            f"Y {gy_value:+.2f} °/s\n"
            f"Z {gz_value:+.2f} °/s\n\n"

            "R = zero   C = calibrate   G = guided",

            fontsize=9
        )


        # ====================================================
        # EMG
        # ====================================================

        if len(samples) > 1:

            xmin = samples[0]
            xmax = samples[-1]

            if xmax <= xmin:

                xmax = xmin + 1


            for channel in range(8):

                values = emg_data[
                    channel
                ]

                if len(values) != len(samples):

                    continue


                emg_lines[
                    channel
                ].set_data(
                    samples,
                    values
                )


                emg_axes[
                    channel
                ].set_xlim(
                    xmin,
                    xmax
                )


                # ============================================
                # AUTOMATIC Y SCALE
                # ============================================

                arr = np.asarray(
                    values,
                    dtype=float
                )

                finite = arr[
                    np.isfinite(arr)
                ]

                if finite.size > 0:

                    low = float(
                        np.min(finite)
                    )

                    high = float(
                        np.max(finite)
                    )

                    span = high - low

                    if span < 0.1:

                        span = 1.0

                    padding = (
                        span *
                        0.10
                    )

                    emg_axes[
                        channel
                    ].set_ylim(
                        low - padding,
                        high + padding
                    )


        # ====================================================
        # WINDOW TITLE
        # ====================================================

        fig.suptitle(

            "PiEEG_XR - "
            "109-byte BLE - "
            "3D IMU + "
            "8 Channel ADS1299 EMG (µV)\n"

            f"Packets: {packets}     "
            f"Bad packets: {bad_packets}     "
            f"EMG samples: {len(samples)}",

            fontsize=13
        )


        return (
            body,
            body_x,
            body_y,
            body_z,
            hand_marker,
            motion_text,
            *emg_lines
        )


    animation = FuncAnimation(

        fig,

        animate,

        interval=33,

        blit=False,

        cache_frame_data=False
    )


    try:

        plt.show()

    finally:

        stop_event.set()


# ============================================================
# RUN
# ============================================================

if __name__ == "__main__":

    main()
