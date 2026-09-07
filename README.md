# Lift Button Controller — OpenMV AE3 + Piper L

Autonomous lift (elevator) button pressing system using an **OpenMV AE3** camera for vision and an **AgileX Piper L** robotic arm for manipulation.

## System Overview

```
┌─────────────────────────┐         WiFi (TCP/JSON)         ┌──────────────────────────┐
│    OpenMV AE3 Camera    │ ◄─────────────────────────────► │    Robot Computer        │
│                         │                                 │    (192.168.123.100)     │
│  • PAG7936 camera       │                                 │                          │
│  • VL53L8CX 8×8 ToF     │                                 │  • Piper L Arm Controller│
│  • Button detection     │                                 │  • CAN bus → Arm         │
│  • Colour classification│                                 │  • Motion planning       │
│  • 3D position output   │                                 │  • Retry logic           │
└─────────────────────────┘                                 └──────────────────────────┘
```

## Directory Structure

```
openmv-piperarm/
├── openmv_camera/                  # Runs on OpenMV AE3 (MicroPython)
│   ├── main.py                     # Entry point — WiFi + TCP server + scan loop
│   ├── config.py                   # WiFi, camera, detection constants
│   ├── button_detector.py          # Vision pipeline (blobs + colour + templates)
│   ├── tof_reader.py               # VL53L8CX time-of-flight wrapper
│   └── templates/                  # Button template images (.pgm)
│       └── README.md               # How to capture templates
│
├── piper_arm/                      # Runs on robot computer (Python 3)
│   ├── run_controller.py           # CLI entry point
│   ├── arm_controller.py           # Workflow orchestrator
│   ├── camera_client.py            # TCP client for OpenMV
│   ├── coordinate_transform.py     # Camera→arm frame math
│   ├── position_recorder.py        # Teach-and-record positions
│   ├── config.yaml                 # Configuration
│   └── positions/                  # Recorded position YAML files
│
└── references/                     # SDK references
    ├── agx_arm_urdf/               # Piper L URDF model
    ├── openmv-python/              # OpenMV Python library
    └── piper_sdk/                  # Legacy Piper SDK (reference only)
```

## Quick Start

### 1. Set Up the OpenMV Camera

1. Connect the OpenMV AE3 to your PC via USB.
2. Open **OpenMV IDE**.
3. Copy the entire `openmv_camera/` directory to the camera's flash filesystem.
4. **Capture template images** following the instructions in `openmv_camera/templates/README.md`.
5. Adjust thresholds in `config.py` for your specific lift panel and lighting.
6. Disconnect USB — the camera will auto-run `main.py` on boot and connect to WiFi.

### 2. Set Up the Robot Computer

**Prerequisites:**
```bash
# Install CAN tools
sudo apt update && sudo apt install can-utils ethtool

# Install Python dependencies
pip3 install python-can pyyaml
pip3 install "git+https://github.com/agilexrobotics/pyAgxArm.git"
```

**Activate the CAN bus:**
```bash
# See: https://github.com/agilexrobotics/pyAgxArm/blob/master/docs/can_user.md
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

**Record starting positions** (with the arm motors disabled for manual teaching):
```bash
cd piper_arm/

# Record the "call-lift" position
python3 run_controller.py record --name call-lift

# Record floor selection positions
python3 run_controller.py record --name floor-select-panel1
python3 run_controller.py record --name floor-select-panel2
```

### 3. Press a Button

```bash
cd piper_arm/

openmv camera IP:  192.168.123.152 

# Press button "2" starting from the floor-select-panel1 position
python3 run_controller.py press \
    --target 2 \
    --start-pos floor-select-panel1 \
    --camera-ip  192.168.123.152 

# Press the "up" arrow from the call-lift position
python3 run_controller.py press \
    --target up \
    --start-pos call-lift \
    --camera-ip  192.168.123.152 

# Dry run (no actual motion — logs commands)
python3 run_controller.py press \
    --target 2 \
    --start-pos floor-select-panel1 \
    --camera-ip  192.168.123.152  \
    --dry-run
```

### 4. Check Status

```bash
python3 run_controller.py status --camera-ip <OPENMV_IP>
```

## Workflow

1. **Move** arm to starting position (joint interpolation)
2. **Scan** — camera captures image, detects buttons, classifies colour, measures ToF distance
3. **Transform** — camera-frame 3D coordinates → arm base frame
4. **Approach** — MoveJ to pre-press standoff position
5. **Press** — MoveL (linear) to push the button
6. **Retract** — MoveL back to standoff
7. **Verify** — re-scan and check if button turned white
8. **Retry** — if not white, repeat up to 3 times
9. **Return** — move back to starting position

## Button Detection Details

| Button | Template File     | Colour States                          |
|--------|-------------------|----------------------------------------|
| `1`    | `btn_1.pgm`       | `green_lit` (always), `white_lit`, `dark` |
| `2`    | `btn_2.pgm`       | `white_lit`, `dark`                    |
| `3`    | `btn_3.pgm`       | `white_lit`, `dark`                    |
| `up`   | `arrow_up.pgm`    | `white_lit`, `dark`                    |
| `down` | `arrow_down.pgm`  | `white_lit`, `dark`                    |

- **`white_lit`** = button successfully pressed (white backlight)
- **`green_lit`** = green backlight (button 1 is always green — ignored)
- **`dark`** = button not pressed / inactive

## Configuration

### OpenMV Camera (`openmv_camera/config.py`)
- WiFi SSID/password
- Camera intrinsics (focal length, principal point)
- LAB colour thresholds for white/green/dark classification
- Blob detection parameters
- Template matching threshold

### Arm Controller (`piper_arm/config.yaml`)
- CAN channel, interface type, and bitrate (pyAgxArm)
- Firmware version selection
- Camera-to-end-effector transform (translation + rotation)
- Tool offset (AgileX 2-finger gripper tip length)
- Panel mapping (position → panel layout)
- Motion speeds (approach, press, retract)
- Press depth and standoff distance
- Retry count

## Communication Protocol

JSON over TCP (newline-delimited), port 8470.

**Commands (robot → camera):**
```json
{"cmd": "SCAN"}
{"cmd": "STATUS"}
```

**Responses (camera → robot):**
```json
{
  "type": "detections",
  "buttons": [
    {
      "id": "2",
      "pixel_x": 320, "pixel_y": 240,
      "cam_x": 0.015, "cam_y": -0.003, "cam_z": 0.250,
      "distance_mm": 250,
      "state": "dark"
    }
  ],
  "timestamp": 1234567890
}
```

## Hardware

- **Camera:** OpenMV AE3 (Alif E3, PAG7936 sensor, VL53L8CX ToF, WiFi)
- **Arm:** AgileX Piper L (6-DOF, CAN bus, ~700 mm reach)
- **Connection:** USB-to-CAN adapter at 1 Mbit/s
- **WiFi:** `MeiPiehChi` network (`robot2robot`)
