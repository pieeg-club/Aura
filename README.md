# Aura_VR
Movement and Muscle in VR/XR 

<img src="https://github.com/pieeg-club/Aura_VR/raw/main/images/aura_gif.gif" alt="Aura VR demo" width="400">


# Aura VR

**Wearable brain–computer interface (BCI) and EMG/IMU motion capture for VR and VRChat.**

<p align="center">
  <a href="https://youtu.be/KI8ROgohrF4">
    <img src="https://img.youtube.com/vi/KI8ROgohrF4/maxresdefault.jpg" alt="Watch the demo on YouTube" width="70%">
  </a>
</p>

Aura VR is an open hardware + firmware + SDK platform built around the
**STM32WB55** wireless MCU and **Texas Instruments ADS1299** 24-bit biosignal
front‑ends. It streams **muscle activity (EMG)** and **motion (accelerometer +
gyroscope)** over Bluetooth Low Energy, so a person's real arm and leg movement
can drive an avatar in **VR / VRChat** in real time.

> The board can run as a full **16‑channel** biosignal recorder (`IronBCI‑16`),
> but for the XR / VR use case you typically start small — **4 EMG channels**
> for the muscles you care about, plus the on‑board IMU for limb orientation.

---

## What it does

- **Reads EMG** from the muscles of the arms and legs with a research‑grade
  24‑bit ADS1299 front‑end (the same class of chip used for EEG).
- **Reads motion** from an on‑board **LSM6DS3** 6‑axis IMU (3‑axis accelerometer
  + 3‑axis gyroscope) to track limb orientation and movement.
- **Streams everything over BLE** to a host (PC / Raspberry Pi) in a single
  compact packet.
- **Feeds VR**: the IMU stream animates a 3D body / limb model; the EMG stream
  reports muscle contraction (e.g. clenching, flexing, gripping).
- **Bridges to VRChat** (roadmap): a *PiEEG Server* forwards movement and muscle
  activity into VRChat via OSC so an avatar mirrors the wearer's real motion and
  effort.

Typical applications: embodied VR / VRChat avatars, gesture and grip control,
rehabilitation and biofeedback, and general EMG/EEG research.

---

## Hardware

Two board families share the same firmware base and BLE protocol:

| Board | MCU | Analog front‑end | Channels | Motion | Best for |
|-------|-----|------------------|----------|--------|----------|
| **Aura VR** | STM32WB55 | 1× ADS1299 | 8 EMG/EEG | LSM6DS3 IMU (accel + gyro) | VR / VRChat, EMG + motion |

Additional sensors present on the design: **MAX30102** (optical PPG / heart‑rate,
optional).

**Key specs (ADS1299)**
- Resolution: 24‑bit, signed
- Sample rate: **250 SPS** (`CONFIG1 = 0x96`)
- Reference: `VREF = 4.5 V`, gain = 1 → **1 LSB ≈ 0.536 µV**

**IMU (LSM6DS3)**
- Accelerometer: ±2 g
- Gyroscope: 245 dps
- Output data rate: 416 Hz (`set_speed = 0x60`)

Schematics, Gerber/DipTrace design files, BOMs and the compiled `.hex` are in the
board folders (`ironbci_16/`, `EMG_VR/`).

---

## BLE protocol

**Advertised name:** `Aura VR` 

| Role | UUID |
|------|------|
| Service | `0000fe40-cc7a-482a-984a-7f2ed5b3e58f` |
| Notify (data) | `0000fe42-8e22-4541-9d4c-21edae82ed19` |
| Write (command) | `0000fe41-8e22-4541-9d4c-21edae82ed19` |

Streaming starts automatically when the host subscribes to notifications.

### Aura VR packet — 109 bytes (EMG + IMU)

```
byte  0        IMU status
bytes 1..6     accelerometer  X, Y, Z   (int16, little‑endian)
bytes 7..12    gyroscope      X, Y, Z   (int16, little‑endian)
bytes 13..36   EMG sample 1   CH1..CH8  (8 ch × 3 bytes, 24‑bit)
bytes 37..60   EMG sample 2   CH1..CH8
bytes 61..84   EMG sample 3   CH1..CH8
bytes 85..108  EMG sample 4   CH1..CH8
```


EMG/EEG channels are big‑endian signed 24‑bit. Convert one channel to
microvolts:

```python
raw = (b0 << 16) | (b1 << 8) | b2              # 3 bytes, MSB first
if raw & 0x800000:                             # 24-bit two's complement
    raw -= 1 << 24
microvolts = 1_000_000 * 4.5 * raw / (2**23 - 1)
```

IMU axes are int16 little‑endian; scale by the configured full‑scale ranges
(±2 g accel, 245 dps gyro).

---

## Repository layout

```
EMG_VR/                         Aura VR (EMG + IMU) — the VR path
├── Framework/                  STM32 firmware sources (p2p_server*.c)
├── project/EMG_VR.rar          Full STM32CubeIDE project (Aura VR)
└── SDK/                        Python host software
    ├── 1_Visualisation_Graph.py   Live accelerometer + gyroscope plot
    ├── 2_body_rotation.py         3D object follows physical IMU rotation
    ├── 3_body_graph.py            IMU plots + 3D object (complementary filter)
    ├── 4_Body_EMG.py              3D IMU orientation + 8‑ch EMG
    ├── 4_1_Body_EMG.py            3D IMU + 8‑ch EMG in µV (109‑byte packet)
    ├── 5.Palm_EMG.py              Hand / palm model driven by EMG (hand_fast.obj)
    └── hand_fast.obj              3D hand mesh


```

---

## Quick start

### 1. Requirements

Host: Windows / Linux / macOS with a BLE adapter (or a Raspberry Pi).

```bash
python -m pip install bleak numpy matplotlib
# fast 16-channel plotter also needs:
python -m pip install pyqtgraph PyQt5
```

### 2. Power the board and start streaming

Power the board; it advertises as `Aura VR`. No pairing PIN is
required — the host just subscribes to the notify characteristic.

### 3. Run a visualizer

**Live IMU (accelerometer + gyroscope):**
```bash
python EMG_VR/SDK/1_Visualisation_Graph.py
```

**3D orientation — the model follows your real movement:**
```bash
python EMG_VR/SDK/2_body_rotation.py
# press R to reset the current pose to zero
```

**EMG + 3D motion together:**
```bash
python EMG_VR/SDK/4_1_Body_EMG.py
```

**16‑channel recorder (IronBCI‑16):**
```bash
python ironbci_16/SDK/1.Python.py                     # scan + plot
python ironbci_16/SDK/1.Python.py --seconds 10 --csv out.csv   # log to CSV
python ironbci_16/SDK/2.Python.py                     # fast pyqtgraph plot
```

Connect by address to skip the name scan:
```bash
python EMG_VR/SDK/2_body_rotation.py --address AA:BB:CC:DD:EE:FF
```

---

## VR / VRChat integration

The XR pipeline turns two physical signals into avatar behaviour:

- **Motion → limb orientation.** The IMU (accelerometer + gyroscope) is fused
  (complementary filter → pitch / roll / yaw) and mapped onto a 3D limb so the
  avatar's arm or leg follows the real one.
- **EMG → muscle activity.** The rectified/smoothed EMG envelope reports how hard
  a muscle is contracting — useful for grip, flex, and gesture triggers on arms
  and legs.

### PiEEG Server → VRChat (roadmap)

The intended VRChat path is a lightweight **PiEEG Server** that:

1. Connects to the board over BLE and decodes the packet above.
2. Converts IMU → joint rotations and EMG → normalized muscle‑activation values.
3. Sends them to VRChat over **OSC** (`/avatar/parameters/...`) so an avatar
   mirrors the wearer's real movement and muscle effort in real time.

> **Status:** the BLE acquisition and 3D/EMG visualization are implemented in the
> `SDK/` scripts. The OSC bridge to VRChat is the next milestone and is not yet
> included in this repository.

---

## Starting minimal: 4 EMG channels

You don't need all 16 (or even all 8) channels to get moving. A common starting
configuration is **4 EMG channels** on the muscles you're targeting (for example
forearm flexor/extensor for grip, plus biceps or a leg muscle), together with the
IMU for limb orientation. Read the full packet as usual and simply use the subset
of channels you've wired up; unused channels can be left unconnected or ignored in
the host script.

---

## Roadmap

- [x] BLE acquisition (8‑ch EMG + IMU, and 16‑ch EEG/EMG)
- [x] Live IMU plots and 3D orientation viewer
- [x] EMG visualization in microvolts + hand/palm model
- [ ] PiEEG Server: OSC bridge to VRChat (movement + muscle activity → avatar)
- [ ] Configurable channel selection / gain from the host
- [ ] Calibration and per‑muscle activation mapping presets

---

## Safety

This is research and hobbyist hardware, **not a medical device**. Use only with
appropriate, isolated power when placing electrodes on the body, and do not use it
for diagnosis or treatment.

---

## Acknowledgements

Built on the PiEEG / IronBCI biosignal platform (STM32WB55 + ADS1299 + LSM6DS3).
See the board folders for schematics, BOMs and firmware.
