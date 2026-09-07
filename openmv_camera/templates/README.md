# Template Images for Button Detection
# ======================================

## Overview

This directory should contain `.pgm` (grayscale PGM) template images of each
lift button type.  The OpenMV `find_template()` function uses normalised cross-
correlation (NCC) to match these templates against the live camera feed.

## Required Templates

| Filename        | Description                           |
|-----------------|---------------------------------------|
| `btn_1.pgm`    | Number "1" button face                |
| `btn_2.pgm`    | Number "2" button face                |
| `btn_3.pgm`    | Number "3" button face                |
| `arrow_up.pgm` | Up arrow (▲ / △) button face          |
| `arrow_down.pgm`| Down arrow (▼ / ▽) button face       |

## How to Capture Templates

1. **Connect the OpenMV AE3 to OpenMV IDE** via USB.
2. **Run a simple snapshot script** (see below) to display the live camera feed.
3. **Position the camera** approximately 20–30 cm from the lift panel, centred
   on one button.
4. **Freeze a frame** in the IDE (click the frame buffer).
5. **Draw a selection rectangle** tightly around the button symbol (number or
   arrow) — exclude the chrome ring.
6. **Save selection** as a `.pgm` file with the correct filename from the table
   above.
7. **Resize** if needed — templates should be roughly 30×30 to 60×60 pixels.
   Smaller templates match faster.
8. **Copy** the `.pgm` files into this `/templates` directory on the OpenMV
   camera's flash filesystem.

### Snapshot Script for Capturing

```python
import csi

cam = csi.CSI()
cam.reset()
cam.pixformat(csi.RGB565)
cam.framesize(csi.VGA)
cam.snapshot(time=2000)  # auto-exposure

while True:
    img = cam.snapshot()
```

## Tips

- Use **grayscale** templates (PGM) even though the camera captures RGB565.
  The `find_template()` function handles the conversion automatically.
- **Consistent lighting** during capture helps — capture under the same
  corridor lighting that the system will operate in.
- If buttons look different when lit vs. unlit, capture the **unlit** version
  (the symbol shape doesn't change; colour classification is handled
  separately).
- You can adjust `TEMPLATE_THRESHOLD` in `config.py` if you get false
  positives (raise threshold) or missed detections (lower threshold).
