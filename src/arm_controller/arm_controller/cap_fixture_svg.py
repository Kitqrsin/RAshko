"""Generate a right-handed, print-at-100%-scale ArUco fixture ring."""

from __future__ import annotations

import argparse
import base64
from pathlib import Path

import cv2

from arm_controller.cap_workcell_config import (
    InspectionSettings,
    load_inspection_settings,
)
from arm_controller.cap_motion_safety import (
    FIXTURE_COORDINATE_CONVENTION,
    require_right_handed_fixture,
)


def _marker_png(marker_id: int, dictionary_id: int) -> bytes:
    aruco = getattr(cv2, "aruco", None)
    if aruco is None:
        raise RuntimeError("OpenCV ArUco support is required")
    try:
        dictionary = aruco.getPredefinedDictionary(dictionary_id)
    except cv2.error as exc:
        raise RuntimeError(f"unsupported ArUco dictionary ID {dictionary_id}") from exc
    marker_count = len(dictionary.bytesList)
    if marker_id < 0 or marker_id >= marker_count:
        raise ValueError(
            f"ArUco marker ID {marker_id} is outside this dictionary's "
            f"0..{marker_count - 1} range"
        )
    if hasattr(aruco, "generateImageMarker"):
        marker = aruco.generateImageMarker(dictionary, marker_id, 600)
    else:  # pragma: no cover - OpenCV 4.6 compatibility
        marker = aruco.drawMarker(dictionary, marker_id, 600)
    success, encoded = cv2.imencode(".png", marker)
    if not success:
        raise RuntimeError(f"could not encode ArUco marker {marker_id}")
    return encoded.tobytes()


def generate_fixture_svg(settings: InspectionSettings) -> str:
    """Return an exact-size SVG for a right-handed, +Z-up fixture frame."""

    require_right_handed_fixture(settings)
    x_min, y_min, x_max, y_max = settings.layout.workspace_bounds_m
    margin_mm = 12.0
    footer_mm = 12.0
    workspace_width_mm = (x_max - x_min) * 1000.0
    workspace_height_mm = (y_max - y_min) * 1000.0
    page_width_mm = workspace_width_mm + 2.0 * margin_mm
    page_height_mm = workspace_height_mm + 2.0 * margin_mm + footer_mm
    parts = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        (
            f'<svg xmlns="http://www.w3.org/2000/svg" '
            f'xmlns:xlink="http://www.w3.org/1999/xlink" '
            f'width="{page_width_mm:.3f}mm" height="{page_height_mm:.3f}mm" '
            f'viewBox="0 0 {page_width_mm:.3f} {page_height_mm:.3f}" '
            f'data-fixture-convention="{FIXTURE_COORDINATE_CONVENTION}">'
        ),
        '<rect width="100%" height="100%" fill="white"/>',
        (
            f'<rect x="{margin_mm:.3f}" y="{margin_mm:.3f}" '
            f'width="{workspace_width_mm:.3f}" '
            f'height="{workspace_height_mm:.3f}" fill="none" '
            'stroke="#777" stroke-width="0.25"/>'
        ),
    ]
    tolerance_m = 1e-6
    for marker_id, corners in sorted(settings.layout.marker_corners_m.items()):
        top_left, top_right, bottom_right, bottom_left = corners
        width_m = top_right[0] - top_left[0]
        # SVG/page Y grows downward.  Fixture Y grows toward the page top so
        # +X cross +Y points out of the printed face (+Z), as ROS expects.
        height_m = top_left[1] - bottom_left[1]
        axis_aligned = (
            width_m > 0.0
            and height_m > 0.0
            and abs(top_right[1] - top_left[1]) <= tolerance_m
            and abs(bottom_right[0] - top_right[0]) <= tolerance_m
            and abs(bottom_left[0] - top_left[0]) <= tolerance_m
            and abs(bottom_right[1] - bottom_left[1]) <= tolerance_m
            and abs(width_m - height_m) <= tolerance_m
        )
        if not axis_aligned:
            raise ValueError(
                f"marker {marker_id} must be an axis-aligned square in "
                "TL, TR, BR, BL order for SVG generation"
            )
        if any(
            not (
                x_min - tolerance_m <= point[0] <= x_max + tolerance_m
                and y_min - tolerance_m <= point[1] <= y_max + tolerance_m
            )
            for point in corners
        ):
            raise ValueError(f"marker {marker_id} leaves the fixture workspace")
        x_mm = margin_mm + (top_left[0] - x_min) * 1000.0
        y_mm = margin_mm + (y_max - top_left[1]) * 1000.0
        size_mm = width_m * 1000.0
        payload = base64.b64encode(
            _marker_png(marker_id, settings.inspector.aruco_dictionary_id)
        ).decode("ascii")
        parts.extend(
            (
                f'<g><title>ArUco {marker_id}</title>',
                (
                    f'<image x="{x_mm:.3f}" y="{y_mm:.3f}" '
                    f'width="{size_mm:.3f}" height="{size_mm:.3f}" '
                    f'preserveAspectRatio="none" image-rendering="pixelated" '
                    f'xlink:href="data:image/png;base64,{payload}"/>'
                ),
                '</g>',
            )
        )
    scale_y = margin_mm + workspace_height_mm + 6.0
    scale_length_mm = min(100.0, workspace_width_mm)
    parts.extend(
        (
            (
                f'<line x1="{margin_mm:.3f}" y1="{scale_y:.3f}" '
                f'x2="{margin_mm + scale_length_mm:.3f}" y2="{scale_y:.3f}" '
                'stroke="black" stroke-width="0.5"/>'
            ),
            (
                f'<text x="{margin_mm:.3f}" y="{scale_y + 4.0:.3f}" '
                'font-family="sans-serif" font-size="3">'
                f'Verify this line is {scale_length_mm:.1f} mm; print at 100%'
                '</text>'
            ),
            (
                f'<text x="{margin_mm:.3f}" y="{scale_y + 10.0:.3f}" '
                'font-family="sans-serif" font-size="3">'
                'Fixture frame: origin lower-left; +X right; +Y page up; '
                '+Z out of printed face'
                '</text>'
            ),
            '</svg>',
        )
    )
    return "\n".join(parts)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--allow-example",
        action="store_true",
        help="allow generation from illustrative example measurements",
    )
    arguments = parser.parse_args(argv)
    settings = load_inspection_settings(arguments.config)
    if settings.example_only and not arguments.allow_example:
        raise SystemExit(
            "refusing to generate a physical fixture from example_only data"
        )
    output = Path(arguments.output)
    output.write_text(generate_fixture_svg(settings), encoding="utf-8")
    print(f"Wrote {output}; print at 100% and verify the scale line")


if __name__ == "__main__":
    main()
