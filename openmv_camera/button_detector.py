# OpenMV AE3 — Lift Button Detector
# ==================================
# Vision pipeline: template matching → colour classification → 3D back-projection.
#
# NEW: the blob pre-filter stage is bypassed. On this board it produced a
# single ~196k-pixel whole-frame blob (LAB threshold far too permissive) and
# a negative roundness value, so every candidate was rejected before template
# matching ever ran. Direct full-frame find_template was proven reliable in
# the IDE, so that's what this uses.
#
# The blob path is kept below as _detect_via_blobs() for when the numbered
# buttons come back and the LAB thresholds have been tuned properly.

import image
import os
import config


class ButtonDetector:
    """Detects lift panel buttons in an RGB565 image frame."""

    def __init__(self):
        self._templates = {}  # {button_id: image.Image}
        self._load_templates()

    # -- Template Loading ------------------------------------------------------

    def _load_templates(self):
        """Load .pgm template images from flash.

        NOTE: find_template() needs a loaded image.Image, not a path string.
        """
        for btn_id, fname in config.BUTTON_TEMPLATES.items():
            path = config.TEMPLATE_DIR + "/" + fname
            try:
                os.stat(path)
                self._templates[btn_id] = image.Image(path)
                print("[Det] Template loaded:", btn_id, "->", path)
            except OSError:
                print("[Det] Template missing:", path)
            except Exception as e:
                print("[Det] Template load failed:", path, e)

    # -- Main Detection Entry Point --------------------------------------------

    def detect(self, img, tof_reader=None, panel=None):
        """Run template matching on the full frame.

        Args:
            img: The captured image.
            tof_reader: Optional ToFReader instance for distance measurement.
            panel: Unused in this path. Kept for API compatibility -- the
                   positional fallback only applies to the blob pipeline.

        Returns a list of dicts:
            [{"id": str, "pixel_x": int, "pixel_y": int,
              "cam_x": float, "cam_y": float, "cam_z": float,
              "distance_mm": int, "state": str}, ...]
        """
        detections = []

        # Template matching needs grayscale. If the frame is RGB565, match
        # against a grayscale copy but sample colour from the original.
        match_img = img
        if img.format() != image.GRAYSCALE:
            try:
                match_img = img.to_grayscale(copy=True)
            except Exception:
                # If copy isn't supported, fall back to matching on img as-is.
                match_img = img

        for btn_id, tpl in self._templates.items():
            try:
                match = match_img.find_template(
                    tpl,
                    config.TEMPLATE_THRESHOLD,
                    step=config.TEMPLATE_STEP,
                    search=image.SEARCH_EX,
                    roi=config.TEMPLATE_ROI,
                )
            except Exception as e:
                print("[Det] Match error for", btn_id, ":", e)
                continue

            if match is None:
                continue

            # find_template returns (x, y, w, h)
            x, y, w, h = match[0], match[1], match[2], match[3]
            cx = x + w // 2
            cy = y + h // 2
            
            if config.TEMPLATE_ROI:
                rx, ry, rw, rh = config.TEMPLATE_ROI
                if not (rx <= cx < rx + rw and ry <= cy < ry + rh):
                    continue
                

            state = self._classify_colour_rect(img, x, y, w, h)

            distance_mm = 0
            if tof_reader is not None:
            #     distance_mm = tof_reader.get_distance_at_pixel(cx, cy)
                  distance_mm = tof_reader.get_distance_in_rect(x, y, w, h)
            cam_x, cam_y, cam_z = self._pixel_to_camera(cx, cy, distance_mm)

            detections.append({
                "id": btn_id,
                "pixel_x": cx,
                "pixel_y": cy,
                "bbox": [x, y, w, h],
                "cam_x": round(cam_x, 4),
                "cam_y": round(cam_y, 4),
                "cam_z": round(cam_z, 4),
                "distance_mm": distance_mm,
                "state": state,
            })

        return detections

    # -- Colour Classification --------------------------------------------------

    def _classify_colour_rect(self, img, x, y, w, h):
        """Classify button state from the average colour inside a rectangle.

        Same thresholds as the blob version, but takes an explicit rect
        instead of a blob object.

        Returns one of: "white_lit", "green_lit", "dark".
        """
        margin = config.BLOB_MARGIN
        rx = max(0, x + margin)
        ry = max(0, y + margin)
        rw = max(1, w - 2 * margin)
        rh = max(1, h - 2 * margin)

        # Clamp to image bounds -- img.width()/height() ARE methods on this
        # firmware, unlike blob attributes.
        rw = min(rw, img.width() - rx)
        rh = min(rh, img.height() - ry)
        if rw <= 0 or rh <= 0:
            return "dark"

        try:
            stats = img.get_statistics(roi=(rx, ry, rw, rh))
            l_mean = stats.l_mean
            a_mean = stats.a_mean
            b_mean = stats.b_mean
        except Exception as e:
            print("[Det] Colour stats failed:", e)
            return "dark"

        if (config.GREEN_THRESH[0] <= l_mean <= config.GREEN_THRESH[1] and
                config.GREEN_THRESH[2] <= a_mean <= config.GREEN_THRESH[3] and
                config.GREEN_THRESH[4] <= b_mean <= config.GREEN_THRESH[5]):
            return "green_lit"

        if (config.WHITE_THRESH[0] <= l_mean <= config.WHITE_THRESH[1] and
                config.WHITE_THRESH[2] <= a_mean <= config.WHITE_THRESH[3] and
                config.WHITE_THRESH[4] <= b_mean <= config.WHITE_THRESH[5]):
            return "white_lit"

        return "dark"

    # -- 3D Back-Projection ------------------------------------------------------

    @staticmethod
    def _pixel_to_camera(px, py, distance_mm):
        """Convert pixel + ToF distance to 3D point in camera frame (metres).

        Pinhole model:
            X_cam = (px - cx) * Z / fx
            Y_cam = (py - cy) * Z / fy
            Z_cam = distance (from ToF)
        """
        if distance_mm <= 0:
            return (0.0, 0.0, 0.0)

        z_m = distance_mm / 1000.0
        fx = config.FOCAL_LENGTH_PX
        fy = config.FOCAL_LENGTH_PX  # assume square pixels
        cx = config.PRINCIPAL_X
        cy = config.PRINCIPAL_Y

        x_m = (px - cx) * z_m / fx
        y_m = (py - cy) * z_m / fy

        return (x_m, y_m, z_m)