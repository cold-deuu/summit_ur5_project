#!/usr/bin/env python3
"""Check the fixed cached values and their historical log evidence, without ROS."""

import importlib.util
import json
from pathlib import Path
import tempfile

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("cached_detection", ROOT / "src/task_planner/src/task_planner/cached_detection.py")
cached = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cached)


def check():
    path = ROOT / "src/task_planner/config/log_detections.json"
    config = cached.load_cached_detections(path)
    expected = [[0.216, -0.620, 1.043], [0.408, -0.639, 1.078]]
    assert [target["position"] for target in config["targets"]] == expected
    assert [target["destination"] for target in config["targets"]] == ["place", "discard"]
    run = ROOT.parent / config["source_run"]
    for target in config["targets"]:
        if run.exists():
            line = target["source_position"]
            text = (run / line["file"]).read_text().splitlines()[line["line"] - 1]
            assert target["source_object"] in text
            assert "(%.3f, %.3f, %.3f)" % tuple(target["position"]) in text
            evidence = target["source_ripe"]
            text = (run / evidence["file"]).read_text().splitlines()[evidence["line"] - 1]
            assert target["source_object"] in text and "property 0" in text
            evidence = target["source_pick_success"]
            text = (run / evidence["file"]).read_text().splitlines()[evidence["line"] - 1]
            assert "Picking completed successfully" in text
    with tempfile.TemporaryDirectory() as directory:
        invalid = Path(directory) / "invalid.json"
        bad = json.loads(path.read_text())
        bad["targets"][0]["position"][0] = float("nan")
        invalid.write_text(json.dumps(bad))
        try:
            cached.load_cached_detections(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid fixed coordinates were accepted")
    print("PASS: exact fixed tomato1/tomato2 coordinates, place/discard roles, ripe and successful-pick log evidence, invalid data rejection")


if __name__ == "__main__":
    check()
