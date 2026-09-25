# Lift Button Controller — OpenMV AE3 + Piper L

Autonomous lift (elevator) button pressing system using an **OpenMV AE3** camera for vision and an **AgileX Piper L** robotic arm for manipulation.

> **Note:** this project originally used WiFi/TCP for the camera link. Everything below
> describes the current **USB serial** architecture, which replaced it for
> reliability.

## System Overview

```
┌─────────────────────────┐      USB Serial (JSON, newline-delimited)     ┌──────────────────────────┐
│    OpenMV AE3 Camera    │ ◄─────────────────────────────────────────►  │    Robot Computer        │
│                         │                                               │                          │
│  • PAG7936 camera       │                                               │  • Piper L Arm Controller│
│  • VL53L8CX 8×8 ToF     │                                               │  • CAN bus → Arm         │
│  • Template matching    │                                               │  • Camera→base transform │
│  • 3D position output   │                                               │  • Motion planning       │
└─────────────────────────┘                                               └──────────────────────────┘
```

No IP address, no WiFi credentials — the camera is found by matching a
substring against `/dev/serial/by-id/`, since `/dev/ttyACM*` numbering isn't
stable across reboots and other USB-serial devices share the bus.

## Directory Structure

```
openmv-piperarm/
├── openmv_camera/                  # Runs on OpenMV AE3 (MicroPython)
│   ├── main.py                     # Entry point — serial protocol loop
│   ├── config.py                   # Camera, ToF, template constants
│   ├── button_detector.py          # Template matching + 3D back-projection
│   ├── tof_reader.py               # VL53L8CX zone mapping + depth lookup
│   └── templates/                  # Button template images (.pgm)
│
├── piper_arm_serial/                # Runs on robot computer (Python 3)
│   ├── run_controller.py           # CLI entry point
│   ├── arm_controller.py           # Workflow orchestrator
│   ├── camera_client.py            # Serial client for OpenMV
│   ├── coordinate_transform.py     # Camera→arm frame math
│   ├── position_recorder.py        # Teach-and-record positions
│   ├── trajectory_play_press.py    # Runs a point sequence + camera-driven press
│   ├── live_button_test.py         # Test transform+press from a manual reading
│   ├── jog_axes.py                 # Calibration aid: find flange axis directions
│   ├── calib_logger.py             # Calibration: log (pose, detection) pairs
│   ├── solve_camera_transform.py   # Calibration: fit tx/ty/tz/rx/ry/rz
│   ├── calib_analyse.py            # Calibration: independent invariance check
│   ├── viewer_grid.py              # Live annotated stream + ToF grid (debugging)
│   ├── config.yaml                 # Configuration
│   ├── positions/                  # Recorded position YAML files
│   └── trajectory/                 # Trajectory sequence files
│
└── references/                     # SDK references
    ├── agx_arm_urdf/               # Piper L URDF model
    └── piper_sdk/                  # Legacy Piper SDK (reference only)
```

## Quick Start

### 1. Set Up the OpenMV Camera

1. Connect the OpenMV AE3 via USB.
2. Push `openmv_camera/*.py` with `mpremote` (**not** the mass-storage
   mount — see [Gotchas](#gotchas)):
   ```bash
   mpremote connect /dev/ttyACM0 fs cp main.py :main.py
   mpremote connect /dev/ttyACM0 fs cp config.py :config.py
   mpremote connect /dev/ttyACM0 fs cp button_detector.py :button_detector.py
   mpremote connect /dev/ttyACM0 fs cp tof_reader.py :tof_reader.py
   mpremote connect /dev/ttyACM0 fs mkdir :/flash/templates
   mpremote connect /dev/ttyACM0 reset
   ```
3. **Capture a template** at your actual working distance and lighting —
   see [Button Detection Details](#button-detection-details).
4. Tune `TEMPLATE_THRESHOLD`/`MIN_PATCH_STDEV` in `config.py` for your
   lighting conditions.

### 2. Set Up the Robot Computer

**Prerequisites:**
```bash
sudo apt update && sudo apt install can-utils
python3 -m venv venv && source venv/bin/activate
pip install pyserial pyyaml numpy scipy
pip install "git+https://github.com/agilexrobotics/pyAgxArm.git"
```

**Activate the CAN bus:**
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

**Record starting positions** (motors disabled for manual teaching):
```bash
cd piper_arm_serial/
python3 run_controller.py record --name home
python3 run_controller.py record --name look-pose
```

### 3. Calibrate the camera-to-flange transform

Required once per physical camera mount — this replaces the old WiFi
version's hardcoded/unmeasured values.
```bash
python3 calib_logger.py --out calib.jsonl --target up
python3 solve_camera_transform.py calib.jsonl --target up
python3 calib_analyse.py calib.jsonl --target up
```
Paste the solved `tx/ty/tz/rx/ry/rz` into `config.yaml`, set
`calibrated: true`. Vary orientation substantially between the 8–10
poses you log, and keep the mobile base (if any) completely still —
see [Gotchas](#gotchas).

### 4. Press a Button

```bash
cd piper_arm_serial/

python3 run_controller.py status

# dry run first -- no motion, just logs the computed target
python3 run_controller.py press --target up --start-pos look-pose --dry-run

# live
python3 run_controller.py press --target up --start-pos look-pose

# or run a full approach/press/retreat sequence from a trajectory file
python3 trajectory_play_press.py --file trajectory/your_file.yaml --speed 15
```

No `--camera-ip` — the serial port is found automatically via `port_hint`
in `config.yaml`.

## Workflow

1. **Move** arm to starting position (joint interpolation)
2. **Scan** — camera captures a frame, template-matches, reads ToF depth
3. **Transform** — camera-frame 3D point → arm base frame
4. **Approach** — point-to-point move to pre-press standoff
5. **Press** — linear move into the button
6. **Retract** — linear move back to standoff
7. **Clear** — extra linear backoff before any joint-space move, so the
   tool doesn't sweep across the panel
8. **Retry** — up to 3 attempts if detection is unreliable at that pose

Visual `white_lit`/`green_lit` verification (step 7 in the original
design) is **currently non-functional** — see
[Known Limitations](#known-limitations).

## Button Detection Details

| Button | Template File | Notes |
|--------|---------------|-------|
| `up`   | operator-captured `.pgm` | actively used; only entry in `BUTTON_TEMPLATES` by default |
| others | — | add by capturing a template at the **same resolution (QVGA) and working distance** detection runs at — `find_template` does no scale search |

Templates must include the button's bezel/border, not just the icon —
an icon alone isn't distinctive enough and false-matches on tape, glare,
or the wrong button.

Colour-based state classification (`white_lit`/`green_lit`/`dark`) is
listed here for completeness but is **not currently functional** — see
[Known Limitations](#known-limitations).

## Configuration

### OpenMV Camera (`openmv_camera/config.py`)
- Camera resolution/intrinsics (QVGA only — see Hardware)
- ToF validity band, axis flips, sensor offset correction
- Template matching threshold and contrast floor (`MIN_PATCH_STDEV`)
- `BUTTON_TEMPLATES` mapping

### Arm Controller (`piper_arm_serial/config.yaml`)
- CAN channel, interface, bitrate, firmware version
- Camera serial `port_hint` (matched against `/dev/serial/by-id/`)
- `camera_transform` — measured, not guessed; see Quick Start §3
- Tool offset (gripper tip length beyond the flange)
- Motion speeds, press depth, approach standoff, retry count
- Scan averaging (`scan_samples`, `scan_min_hits`, `scan_max_spread_mm`)

## Communication Protocol

JSON over USB serial, newline-delimited. No port number, no IP.

**Commands (robot → camera):**
```json
{"cmd": "SCAN"}
{"cmd": "STATUS"}
{"cmd": "FRAME", "quality": 50}
```

**Responses (camera → robot):**
```json
{
  "type": "detections",
  "buttons": [
    {
      "id": "up",
      "pixel_x": 158, "pixel_y": 89,
      "bbox": [141, 72, 35, 35],
      "cam_x": -0.0015, "cam_y": -0.0081, "cam_z": 0.184,
      "distance_mm": 184,
      "state": "dark"
    }
  ],
  "timestamp": 1234567890
}
```
A `{"type": "boot", "message": "main.py running"}` line is sent once on
startup — useful as a machine-checkable liveness signal.

## Hardware

- **Camera:** OpenMV AE3 (Alif Ensemble E3, PAG7936 sensor, VL53L8CX ToF)
- **Arm:** AgileX Piper L (6-DOF, CAN bus)
- **Connection:** USB (camera), USB-to-CAN adapter at 1 Mbit/s (arm)

## Known Limitations

- **`move_l` (linear Cartesian motion) has occasionally silently failed
  to move the arm** — no error raised, arm just doesn't move. Not yet
  isolated; suspected singularity or unreachable IK.
- **Template matching has no scale tolerance** — works only within a
  narrow band around the capture distance.
- **Colour state classification is non-functional.** The camera captures
  grayscale (memory constraints at higher resolutions), which has no
  a/b chrominance channels, so `white_lit` verification can never
  succeed. `verify_press` should stay `false`.
- **Press direction follows the flange's current orientation, not the
  panel's actual surface normal.** Fine when squared to the panel.
- **No lens distortion correction** — positional error increases for
  detections near the frame edges.

## Gotchas

- **Never edit camera files through the mass-storage mount** while
  MicroPython also has the filesystem open — two writers corrupts it.
  Always use `mpremote`.
- **`main.py` owns `sys.stdin`** while running — file pushes need a
  reset first.
- **Only one process can hold the camera's serial port** — stop
  `viewer_grid.py` before running anything that presses.
- **Don't move the mobile base during calibration collection.** The
  method assumes the button's base-frame position is constant across
  samples; base movement corrupts this silently.
- **Avoid hand-editing `.py` files in `nano` over SSH** — pasted blocks
  have repeatedly mixed tabs and spaces here, producing indentation
  errors invisible in the editor. Prefer `scp` or a `cat > file << 'EOF'`
  heredoc, and verify with `python3 -c "import ast; ast.parse(open('f.py').read())"`.