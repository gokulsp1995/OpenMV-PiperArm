# OpenMV AE3 — Lift Button Detector
# ==================================
# Vision pipeline: blob detection → colour classification → template matching.
# Produces a list of detected buttons with ID, pixel position, colour state,
# and 3D camera-frame coordinates (using ToF distance + pinhole model).

import image
import os
import config


class ButtonDetector:
    """Detects lift panel buttons in an RGB565 image frame."""

    def __init__(self):
        self._templates = {}  # {button_id: image.Image}
        self._load_templates()

    # ── Template Loading ──────────────────────────────────────────────────────

    def _load_templates(self):
        """Load .pgm template images from flash."""
        for btn_id, fname in config.BUTTON_TEMPLATES.items():
            path = config.TEMPLATE_DIR + "/" + fname
            try:
                # Check file exists (MicroPython os.stat)
                os.stat(path)
                self._templates[btn_id] = path  # store path; load lazily
                print("[Det] Template loaded:", btn_id, "→", path)
            except OSError:
                print("[Det] Template missing:", path)

    # ── Main Detection Entry Point ────────────────────────────────────────────

    def detect(self, img, tof_reader=None, panel=None):
        """Run the full detection pipeline on *img* (RGB565).

        Args:
            img: The captured RGB565 image.
            tof_reader: Optional ToFReader instance for distance measurement.
            panel: Optional panel name ("panel1" or "panel2") for position-
                   based identification fallback when templates are missing.

        Returns a list of dicts:
            [{"id": str, "pixel_x": int, "pixel_y": int,
              "cam_x": float, "cam_y": float, "cam_z": float,
              "distance_mm": int, "state": str}, ...]
        """
        detections = []

        # --- Step 1: Find candidate blobs (circular bright-ish regions) ------
        blobs = img.find_blobs(
            config.BLOB_THRESH,
            pixels_threshold=config.BLOB_MIN_PIXELS,
            area_threshold=config.BLOB_MIN_PIXELS,
            merge=True,
            margin=config.BLOB_MERGE_DISTANCE,
        )

        # Filter by circularity and size
        candidates = []
        for b in blobs:
            if b.pixels() > config.BLOB_MAX_PIXELS:
                continue
            if b.roundness() < config.BLOB_MIN_CIRCULARITY:
                continue
            candidates.append(b)

        # --- Step 2: For each candidate, classify colour and identify --------
        for blob in candidates:
            cx = blob.cx()
            cy = blob.cy()

            # Colour classification in the blob's ROI
            state = self._classify_colour(img, blob)

            # Template matching to determine button identity
            btn_id = self._identify_button(img, blob)
            if btn_id is None and panel is not None:
                # Defer positional assignment (done after all blobs collected)
                pass
            elif btn_id is None:
                continue  # skip blobs that don't match any known button

            # --- Step 3: Distance from ToF ───────────────────────────────────
            distance_mm = 0
            if tof_reader is not None:
                distance_mm = tof_reader.get_distance_at_pixel(cx, cy)

            # --- Step 4: Back-project to 3D camera frame ─────────────────────
            cam_x, cam_y, cam_z = self._pixel_to_camera(cx, cy, distance_mm)

            detections.append({
                "id": btn_id,
                "pixel_x": cx,
                "pixel_y": cy,
                "cam_x": round(cam_x, 4),
                "cam_y": round(cam_y, 4),
                "cam_z": round(cam_z, 4),
                "distance_mm": distance_mm,
                "state": state,
            })

        # --- Step 5: Positional fallback for unidentified blobs ───────────────
        # If we have a panel layout and some detections lack IDs, try to assign
        # them based on their spatial arrangement.
        unidentified = [d for d in detections if d["id"] is None]
        if unidentified and panel is not None:
            self._assign_ids_by_position(unidentified, panel)

        # Remove any remaining unidentified detections
        detections = [d for d in detections if d["id"] is not None]

        return detections

    # ── Colour Classification ─────────────────────────────────────────────────

    def _classify_colour(self, img, blob):
        """Classify the button state based on average colour inside the blob.

        Returns one of: "white_lit", "green_lit", "dark".
        """
        # Extract a small ROI around the blob centre for colour sampling
        margin = config.BLOB_MARGIN
        rx = max(0, blob.x() + margin)
        ry = max(0, blob.y() + margin)
        rw = max(1, blob.w() - 2 * margin)
        rh = max(1, blob.h() - 2 * margin)
        roi = (rx, ry, rw, rh)

        stats = img.get_statistics(roi=roi)
        l_mean = stats.l_mean()
        a_mean = stats.a_mean()
        b_mean = stats.b_mean()

        # Check green first (button 1 is always green — caller may ignore)
        if (config.GREEN_THRESH[0] <= l_mean <= config.GREEN_THRESH[1] and
                config.GREEN_THRESH[2] <= a_mean <= config.GREEN_THRESH[3] and
                config.GREEN_THRESH[4] <= b_mean <= config.GREEN_THRESH[5]):
            return "green_lit"

        # Check white
        if (config.WHITE_THRESH[0] <= l_mean <= config.WHITE_THRESH[1] and
                config.WHITE_THRESH[2] <= a_mean <= config.WHITE_THRESH[3] and
                config.WHITE_THRESH[4] <= b_mean <= config.WHITE_THRESH[5]):
            return "white_lit"

        # Default: dark / unlit
        return "dark"

    # ── Template Identification ───────────────────────────────────────────────

    def _identify_button(self, img, blob):
        """Try to match the blob region against known button templates.

        Returns the button ID string ("1", "2", "up", …) or None.
        """
        if not self._templates:
            # No templates available — fall back to positional heuristic
            return self._identify_by_position(blob)

        best_id = None
        best_score = config.TEMPLATE_THRESHOLD  # minimum acceptable score

        # Extract the blob ROI as a sub-image for matching
        margin = 2
        rx = max(0, blob.x() - margin)
        ry = max(0, blob.y() - margin)
        rw = min(img.width() - rx, blob.w() + 2 * margin)
        rh = min(img.height() - ry, blob.h() + 2 * margin)
        roi = (rx, ry, rw, rh)

        for btn_id, tpl_path in self._templates.items():
            try:
                # find_template returns the best match within the ROI
                # step=2 for speed, search=image.SEARCH_EX for exhaustive
                match = img.find_template(
                    tpl_path,
                    config.TEMPLATE_THRESHOLD,
                    roi=roi,
                    step=2,
                    search=image.SEARCH_EX,
                )
                if match is not None:
                    # match is (x, y, w, h, score) — newer API
                    # or (x, y, w, h) — older API; score via separate call
                    # We simply accept the first match above threshold.
                    # If we got here, the template matched.
                    # Use overlap ratio as a proxy score
                    # (find_template already filtered by threshold)
                    best_id = btn_id
                    break  # first match wins (templates are distinct enough)
            except Exception:
                pass  # template load/match failure — skip

        return best_id

    def _identify_by_position(self, blob):
        """Fallback heuristic when templates are unavailable.

        This is only used when panel layout is NOT provided.
        When a panel name IS given, _assign_ids_by_position is used instead.
        """
        return None

    def _assign_ids_by_position(self, detections, panel_name):
        """Assign button IDs to unidentified detections based on panel layout.

        Modifies the detection dicts in-place, setting their 'id' field.

        Panel 1 (horizontal row):  sort left→right by pixel_x → (1)(2)(3)
        Panel 2 (3×2 grid):       cluster into grid cells → map by (row,col)
        """
        layout = config.PANEL_LAYOUTS.get(panel_name)
        if layout is None:
            return

        arrangement = layout.get("arrangement", "horizontal")

        if arrangement == "horizontal":
            # Sort by X position (left to right)
            detections.sort(key=lambda d: d["pixel_x"])
            order = layout.get("order", [])
            for i, det in enumerate(detections):
                if i < len(order):
                    det["id"] = order[i]
                # Extra blobs beyond the known buttons stay None

        elif arrangement == "grid":
            grid_map = layout.get("grid", {})
            n_rows = layout.get("rows", 3)
            n_cols = layout.get("cols", 2)

            if not detections:
                return

            # Determine grid boundaries from the blob positions
            xs = [d["pixel_x"] for d in detections]
            ys = [d["pixel_y"] for d in detections]
            x_min, x_max = min(xs), max(xs)
            y_min, y_max = min(ys), max(ys)

            # Add margin to avoid edge effects
            x_span = max(x_max - x_min, 1)
            y_span = max(y_max - y_min, 1)

            for det in detections:
                # Map pixel position to grid cell
                col = int((det["pixel_x"] - x_min) * n_cols / (x_span + 1))
                row = int((det["pixel_y"] - y_min) * n_rows / (y_span + 1))
                col = max(0, min(n_cols - 1, col))
                row = max(0, min(n_rows - 1, row))

                btn_id = grid_map.get((row, col))
                if btn_id is not None:
                    det["id"] = btn_id

    # ── 3D Back-Projection ────────────────────────────────────────────────────

    @staticmethod
    def _pixel_to_camera(px, py, distance_mm):
        """Convert pixel + ToF distance to 3D point in camera frame (metres).

        Uses a simple pinhole camera model:
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
