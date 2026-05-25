#!/usr/bin/env python3
"""
aruco_marker_generator.py
--------------------------
Generates a printable A4 PDF/PNG with 4 ArUco markers placed at the corners
of a rectangle matching the puzzle wall dimensions.

Usage:
    python3 aruco_marker_generator.py

Output:
    puzzle_wall_aruco_sheet.png  — print this at 100% scale (no fit-to-page)
    puzzle_wall_aruco_sheet.pdf  — PDF version

The physical puzzle wall dimensions are set in PUZZLE_WALL_WIDTH_MM and
PUZZLE_WALL_HEIGHT_MM below. Adjust to match your actual puzzle board.

Marker IDs (corners, fixed):
    0 = top-left
    1 = top-right
    2 = bottom-right
    3 = bottom-left

These IDs must match aruco_config.yaml.
"""

import cv2
import numpy as np
import os

# ── Puzzle wall physical dimensions (millimetres) ──────────────────────────
PUZZLE_WALL_WIDTH_MM  = 200   # e.g. 20 cm wide
PUZZLE_WALL_HEIGHT_MM = 200   # e.g. 20 cm tall

# ── Marker settings ─────────────────────────────────────────────────────────
MARKER_SIZE_MM   = 20         # physical side length of each printed marker
MARGIN_MM        = 10         # gap between marker edge and wall corner
ARUCO_DICT_ID    = cv2.aruco.DICT_4X4_50
MARKER_IDS       = [5, 6, 7, 8]   # TL, TR, BR, BL

# ── Output resolution ───────────────────────────────────────────────────────
DPI = 300
MM_PER_INCH = 25.4

def mm_to_px(mm):
    return int(round(mm / MM_PER_INCH * DPI))

def generate_sheet():
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)

    # Canvas size = puzzle wall + margins on all sides
    canvas_w_mm = PUZZLE_WALL_WIDTH_MM  + 2 * MARGIN_MM
    canvas_h_mm = PUZZLE_WALL_HEIGHT_MM + 2 * MARGIN_MM

    canvas_w_px = mm_to_px(canvas_w_mm)
    canvas_h_px = mm_to_px(canvas_h_mm)
    marker_px   = mm_to_px(MARKER_SIZE_MM)

    canvas = np.ones((canvas_h_px, canvas_w_px), dtype=np.uint8) * 255  # white

    # Corner pixel positions (top-left corner of each marker)
    margin_px = mm_to_px(MARGIN_MM)
    positions = {
        5: (margin_px,                          margin_px),                         # TL
        6: (canvas_w_px - margin_px - marker_px, margin_px),                        # TR
        7: (canvas_w_px - margin_px - marker_px, canvas_h_px - margin_px - marker_px),  # BR
        8: (margin_px,                           canvas_h_px - margin_px - marker_px),  # BL
    }

    label_map = {5: "TL (ID 5)", 6: "TR (ID 6)", 7: "BR (ID 7)", 8: "BL (ID 8)"}

    for marker_id in MARKER_IDS:
        marker_img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_px)
        x, y = positions[marker_id]
        canvas[y:y+marker_px, x:x+marker_px] = marker_img

        # Label below marker (small text for reference)
        label = label_map[marker_id]
        font_scale = marker_px / 300.0
        cv2.putText(
            canvas, label,
            (x, y + marker_px + mm_to_px(5)),
            cv2.FONT_HERSHEY_SIMPLEX, font_scale, 0, max(1, mm_to_px(0.4))
        )

    # Draw a thin border representing the puzzle wall boundary
    wall_x0 = mm_to_px(MARGIN_MM)
    wall_y0 = mm_to_px(MARGIN_MM)
    wall_x1 = canvas_w_px - mm_to_px(MARGIN_MM)
    wall_y1 = canvas_h_px - mm_to_px(MARGIN_MM)
    cv2.rectangle(canvas, (wall_x0, wall_y0), (wall_x1, wall_y1), 180, mm_to_px(0.5))

    # Dimension labels
    dim_label = f"Puzzle wall: {PUZZLE_WALL_WIDTH_MM}mm x {PUZZLE_WALL_HEIGHT_MM}mm | Markers: {MARKER_SIZE_MM}mm | Print at 100% scale"
    cv2.putText(canvas, dim_label, (margin_px, canvas_h_px - mm_to_px(3)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, 100, 1)

    out_path = "puzzle_wall_aruco_sheet.png"
    cv2.imwrite(out_path, canvas)
    print(f"[aruco_generator] Saved: {out_path}")
    print(f"  Canvas: {canvas_w_mm:.1f} x {canvas_h_mm:.1f} mm  ({canvas_w_px} x {canvas_h_px} px at {DPI} DPI)")
    print(f"  Puzzle wall region: {PUZZLE_WALL_WIDTH_MM} x {PUZZLE_WALL_HEIGHT_MM} mm")
    print(f"  Marker size: {MARKER_SIZE_MM} mm | IDs: {MARKER_IDS}")
    print()
    print("  IMPORTANT: Print at exactly 100% scale (no 'fit to page').")
    print("  Measure one marker with a ruler after printing to verify.")

    # ── Optional PDF via matplotlib ─────────────────────────────────────────
    try:
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_pdf import PdfPages
        fig, ax = plt.subplots(figsize=(canvas_w_mm/MM_PER_INCH, canvas_h_mm/MM_PER_INCH), dpi=DPI)
        ax.imshow(canvas, cmap='gray', vmin=0, vmax=255)
        ax.axis('off')
        fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
        pdf_path = "puzzle_wall_aruco_sheet.pdf"
        with PdfPages(pdf_path) as pdf:
            pdf.savefig(fig, bbox_inches='tight', pad_inches=0)
        print(f"[aruco_generator] Saved PDF: {pdf_path}")
        plt.close(fig)
    except ImportError:
        print("[aruco_generator] matplotlib not available — PNG only.")

if __name__ == "__main__":
    generate_sheet()