"""Load and validate standard ROS monocular camera-calibration YAML files."""

from __future__ import annotations

import math
from pathlib import Path

import yaml


_MATRIX_FIELDS = {
    "camera_matrix": 9,
    "distortion_coefficients": None,
    "rectification_matrix": 9,
    "projection_matrix": 12,
}


def load_camera_info_yaml(path: str) -> dict:
    """Return validated camera data in ROS ``CameraInfo`` field form."""

    calibration_path = Path(path)
    with calibration_path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    if not isinstance(raw, dict):
        raise ValueError("camera calibration must be a YAML object")

    width = raw.get("image_width")
    height = raw.get("image_height")
    if type(width) is not int or type(height) is not int:
        raise ValueError("camera calibration dimensions must be integers")
    if width <= 0 or height <= 0:
        raise ValueError("camera calibration dimensions must be positive")

    matrices = {}
    for field, expected_length in _MATRIX_FIELDS.items():
        entry = raw.get(field)
        if not isinstance(entry, dict) or not isinstance(entry.get("data"), list):
            raise ValueError(f"camera calibration is missing {field}.data")
        values = tuple(float(value) for value in entry["data"])
        if expected_length is not None and len(values) != expected_length:
            raise ValueError(
                f"{field}.data must contain {expected_length} values"
            )
        if field == "distortion_coefficients" and len(values) == 0:
            raise ValueError("distortion_coefficients.data cannot be empty")
        if any(not math.isfinite(value) for value in values):
            raise ValueError(f"{field}.data contains a non-finite value")
        matrices[field] = values

    if matrices["camera_matrix"][0] <= 0.0 or matrices["camera_matrix"][4] <= 0.0:
        raise ValueError("camera focal lengths fx and fy must be positive")
    distortion_model = str(raw.get("distortion_model", "")).strip()
    supported_lengths = {"plumb_bob": {5}, "rational_polynomial": {8}}
    if distortion_model not in supported_lengths:
        raise ValueError("camera calibration distortion_model is unsupported")
    if len(matrices["distortion_coefficients"]) not in supported_lengths[
        distortion_model
    ]:
        raise ValueError(
            "camera calibration distortion coefficient count is unsupported"
        )

    return {
        "width": width,
        "height": height,
        "distortion_model": distortion_model,
        "d": matrices["distortion_coefficients"],
        "k": matrices["camera_matrix"],
        "r": matrices["rectification_matrix"],
        "p": matrices["projection_matrix"],
        "camera_name": str(raw.get("camera_name", calibration_path.stem)),
        "source": str(calibration_path),
    }


def require_matching_resolution(calibration: dict, width: int, height: int) -> None:
    """Reject runtime images whose geometry differs from calibration."""

    expected = (int(calibration["width"]), int(calibration["height"]))
    actual = (int(width), int(height))
    if actual != expected:
        raise ValueError(
            f"runtime image is {actual[0]}x{actual[1]}, but calibration is "
            f"{expected[0]}x{expected[1]}"
        )
