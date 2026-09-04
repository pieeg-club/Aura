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
# GYRO NOISE DEAD BAND
# ============================================================

def apply_deadband(value):

    if abs(value) < GYRO_DEADBAND:

        return 0.0

    return value


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

    gx = apply_deadband(gx)
    gy = apply_deadband(gy)
    gz = apply_deadband(gz)

    with sensor.lock:

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

        if dt > 0.20:

            return

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
HAND_MAX_FACES = 1200

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
    # KEY R = ZERO
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

                print(
                    "Orientation ZERO"
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

            packets = (
                sensor.packet_counter
            )

            bad_packets = (
                sensor.bad_packet_counter
            )

            connected = (
                sensor.connected
            )

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

        rotated = (
            vertices @
            R.T
        )

        # NumPy advanced indexing is much faster than a Python face loop.
        rotated_faces = rotated[faces_index]

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


        body_x.set_data(
            [0, rx[0]],
            [0, rx[1]]
        )

        body_x.set_3d_properties(
            [0, rx[2]]
        )


        body_y.set_data(
            [0, ry[0]],
            [0, ry[1]]
        )

        body_y.set_3d_properties(
            [0, ry[2]]
        )


        body_z.set_data(
            [0, rz[0]],
            [0, rz[1]]
        )

        body_z.set_3d_properties(
            [0, rz[2]]
        )


        # ====================================================
        # 3D STATUS
        # ====================================================

        if connected:

            status = "BLE CONNECTED"

        else:

            status = "WAITING FOR BLE"


        ax3d.set_title(

            f"{status}\n\n"

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

            "R = zero orientation",

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
            *emg_lines
        )


    animation = FuncAnimation(

        fig,

        animate,

        interval=25,

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
