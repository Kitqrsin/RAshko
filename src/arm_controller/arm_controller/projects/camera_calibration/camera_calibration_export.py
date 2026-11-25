"""Safely export and validate ROS cameracalibrator's saved intrinsic YAML."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import tarfile
import tempfile

from arm_controller.camera_info_loader import load_camera_info_yaml


def export_calibration_archive(
    archive_path: str,
    output_path: str,
    *,
    overwrite: bool = False,
) -> dict:
    """Extract only ``ost.yaml``, validate it, and atomically install it."""

    archive = Path(archive_path)
    output = Path(output_path)
    if not archive.is_file():
        raise ValueError(
            f"calibration archive {archive} does not exist; click SAVE first"
        )
    if output.exists() and not overwrite:
        raise ValueError(
            f"calibration output {output} already exists; use --force to replace it"
        )
    with tarfile.open(archive, "r:gz") as bundle:
        try:
            member = bundle.getmember("ost.yaml")
        except KeyError as exc:
            raise ValueError("calibration archive has no ost.yaml") from exc
        if not member.isfile() or member.size <= 0 or member.size > 1_000_000:
            raise ValueError("calibration ost.yaml has an invalid size")
        stream = bundle.extractfile(member)
        if stream is None:
            raise ValueError("calibration ost.yaml could not be read")
        contents = stream.read()

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary_name = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb",
            prefix=output.name + ".",
            suffix=".tmp",
            dir=output.parent,
            delete=False,
        ) as temporary:
            temporary.write(contents)
            temporary.flush()
            os.fsync(temporary.fileno())
            temporary_name = temporary.name
        calibration = load_camera_info_yaml(temporary_name)
        os.replace(temporary_name, output)
        temporary_name = None
    finally:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)
    return calibration


def main() -> None:
    """Validate arguments and export one camera calibration."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--archive",
        default="/tmp/calibrationdata.tar.gz",
    )
    parser.add_argument(
        "--output",
        default="/workspace/config/camera_intrinsics.yaml",
    )
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    calibration = export_calibration_archive(
        args.archive,
        args.output,
        overwrite=args.force,
    )
    print(
        "Saved validated camera calibration: "
        f"{args.output} ({calibration['width']}x{calibration['height']}, "
        f"{calibration['distortion_model']})"
    )


if __name__ == "__main__":
    main()
