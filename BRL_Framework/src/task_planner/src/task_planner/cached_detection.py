"""Load fixed historical detections; no cameras, YOLO, or live observations."""

import json
import math
from pathlib import Path


def load_cached_detections(path):
    with Path(path).expanduser().open(encoding="utf-8") as stream:
        config = json.load(stream)
    if config.get("mode") != "fixed_log_coordinates" or config.get("frame_id") != "robot_base_footprint":
        raise ValueError("Expected fixed log coordinates in robot_base_footprint")
    targets = config.get("targets", [])
    if len(targets) != 2 or [target.get("name") for target in targets] != ["tomato1", "tomato2"]:
        raise ValueError("Cached detection must define tomato1 and tomato2")
    if [target.get("destination") for target in targets] != ["place", "discard"]:
        raise ValueError("Cached detection must assign tomato1=place, tomato2=discard")
    for target in targets:
        xyz = target.get("position", [])
        if (target.get("tomato_class") != 0 or len(xyz) != 3 or
                not all(isinstance(v, (int, float)) and math.isfinite(v) for v in xyz)):
            raise ValueError("Cached targets require ripe class 0 and finite XYZ")
    return config
