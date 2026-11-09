#!/usr/bin/env python3
"""Offscreen GUI checks; process lifecycle uses a sleeping Python test process."""

import importlib.util
import os
from pathlib import Path
import sys
import tempfile
import time

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QApplication

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("brl_gui", ROOT / "run_master_gui.py")
gui = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gui)


def main():
    app = QApplication(sys.argv[:1])
    with tempfile.TemporaryDirectory(prefix="brl-gui-test-") as directory:
        window = gui.MasterWindow(gui.parse_args(["--no-telemetry", "--log-dir", directory]))
        window.show()
        app.processEvents()
        assert window.styleSheet() == "", "Window overrides the legacy native Qt appearance"
        assert window.size().width() == 1450 and window.size().height() == 900
        assert window.details_panel.isHidden()
        window.details_button.click()
        assert not window.details_panel.isHidden()
        window.details_button.click()
        assert window.details_panel.isHidden()
        assert len(window.preview_actions) == 9
        assert window.plan_combo.isVisible()
        assert [window.plan_combo.itemText(index) for index in range(window.plan_combo.count())] == ["T1", "사용자 지정"]
        assert window.plan_combo.currentText() == "T1"
        assert not window.start_button.isEnabled()
        assert all(card.process is None for card in window.cards)
        assert window.plan_tree.topLevelItem(0).text(1) == "1"
        assert window.preview_actions == [
            "prepare_nav(brl_robot)", "detect(brl_robot, stem_01)",
            "pick(brl_robot, tomato1, stem_01)", "scan(brl_robot, tomato1)",
            "place(brl_robot, tomato1)",
            "pick(brl_robot, tomato2, stem_01)", "scan(brl_robot, tomato2)",
            "discard(brl_robot, tomato2)", "prepare_nav(brl_robot)",
        ]
        invalid = Path(directory) / "invalid.yaml"
        invalid.write_text("offline_actions: []")
        window.plan_path.setText(str(invalid))
        assert not window.preview_actions and not window.load_button.isEnabled()
        quoted = Path(directory) / "plan 'quoted' name.yaml"
        quoted.write_text((gui.PLAN_DIRECTORY / "T1.yaml").read_text())
        window.receive_event({"type": "connection", "connected": True, "planner_ready": True})
        window.receive_event({"type": "status", "status": "running"})
        assert window.plan_combo.isEnabled() and window.plan_path.isEnabled()
        assert not window.load_button.isEnabled() and not window.start_button.isEnabled()
        original_dialog = gui.QFileDialog.getOpenFileName
        try:
            gui.QFileDialog.getOpenFileName = lambda *args: (str(quoted), "YAML")
            window.plan_combo.setCurrentText("T1")
            window.plan_combo.setCurrentText("사용자 지정")
            assert window.plan_path.text() == str(quoted)
            assert window.runtime_status == "running" and all(card.process is None for card in window.cards)
            window.control_busy = True
            window.update_buttons()
            assert window.plan_combo.isEnabled() and window.plan_path.isEnabled()
            window.plan_combo.setCurrentText("T1")
            assert window.plan_path.text() == str(gui.PLAN_DIRECTORY / "T1.yaml")
            gui.QFileDialog.getOpenFileName = lambda *args: ("", "")
            window.plan_combo.setCurrentText("사용자 지정")
            assert window.plan_combo.currentText() == "T1"
        finally:
            gui.QFileDialog.getOpenFileName = original_dialog
        window.control_busy = False
        window.receive_event({"type": "status", "status": "idle"})
        print("PASS: plan selection and path editing stay available while running or busy; custom selection opens the file dialog")
        window.plan_path.setText(str(quoted))
        window.receive_event({"type": "connection", "connected": True, "planner_ready": True})
        assert window.load_button.isEnabled() and window.start_button.isEnabled()
        calls = []
        original_run = window.run_command
        def fake_failure(command, callback=None, **kwargs):
            calls.append(command)
            callback(True, "success: false\nmessage: rejected\n")
        window.run_command = fake_failure
        window.start_button.click()
        assert len(calls) == 1 and calls[0][2] == "/generate_task_plan"
        assert window.runtime_status != "running"
        calls.clear()
        def fake_success(command, callback=None, **kwargs):
            calls.append(command)
            callback(True, "success: true\nmessage: accepted\n")
        window.run_command = fake_success
        window.load_button.click()
        assert len(calls) == 1 and calls[0][2] == "/generate_task_plan"
        assert str(quoted) in calls[0][3]
        assert window.runtime_status == "idle"
        window.start_button.click()
        assert [call[2] for call in calls[-2:]] == ["/generate_task_plan", "/task_planner/start"]
        window.pause_button.click()
        assert window.pause_requested and window.resume_button.isEnabled()
        window.resume_button.click()
        assert not window.pause_requested
        window.stop_button.click()
        assert calls[-1][2] == "/task_planner/stop"
        window.receive_event({"type": "status", "status": "running"})
        window.receive_event({"type": "scene", "environment": "Preview",
            "robots": [{"name": "brl_robot", "status": "active", "location": "Stem_01", "held_objects": ["tomato1"]}],
            "objects": [{"name": "tomato1", "status": "picked", "location": "", "holder": "brl_robot", "class": "ripe"}]})
        window.plan_path.setText(str(gui.PLAN_DIRECTORY / "T1.yaml"))
        window.receive_event({"type": "action", "action": "scan(brl_robot, tomato1)"})
        window.receive_event({"type": "targets", "targets": [
            {"alias": "tomato1", "name": "tomato1", "x": 0.216, "y": -0.620, "z": 1.043,
             "confidence": None, "destination": "place"},
            {"alias": "tomato2", "name": "tomato2", "x": 0.408, "y": -0.639, "z": 1.078,
             "confidence": None, "destination": "discard"}]})
        assert "로그 고정값" in window.target_label.text() and "discard" in window.target_label.text()
        assert window.current_row == 3 and window.scene_tree.topLevelItemCount() == 2
        app.processEvents()
        window.grab().save("/tmp/brl-framework-gui-preview.png")
        window.receive_event({"type": "status", "status": "completed"})
        detect_only = Path(directory) / "detect_only.yaml"
        detect_only.write_text("offline_actions:\n  - index: 1\n    name: detect(brl_robot, stem_01)\n")
        window.plan_path.setText(str(detect_only))
        assert len(window.preview_actions) == 1 and window.preview_actions[0] == "detect(brl_robot, stem_01)"
        def fake_immediate_completion(command, callback=None, **kwargs):
            if command[2] == "/task_planner/start":
                window.receive_event({"type": "status", "status": "completed"})
            callback(True, "success: true\nmessage: accepted\n")
        window.run_command = fake_immediate_completion
        window.start_button.click()
        assert window.runtime_status == "completed" and "완료" in window.status_label.text()
        assert window.start_button.isEnabled() and not window.control_busy
        window.plan_combo.setCurrentText("T1")
        assert len(window.preview_actions) == 9
        assert window.plan_path.text() == str(gui.PLAN_DIRECTORY / "T1.yaml")
        print("PASS: immediate plan completion before the start service reply preserves GUI completion state")
        print("PASS: offline plan preview, invalid plan, quoted path, no auto-start, service rejection, load/start/pause/resume/stop, scene/action view")

        window.run_command = original_run
        window.component_command = lambda name: [sys.executable, "-u", "-c",
            "import time; print('fake ready', flush=True); time.sleep(60)"]
        card = window.cards[0]
        def wait_until(predicate, timeout=10):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                app.processEvents()
                if predicate():
                    return
                time.sleep(0.02)
            raise AssertionError("GUI process lifecycle timed out")
        card.start()
        wait_until(lambda: card.process.state() == QProcess.Running and "fake ready" in card.log.toPlainText())
        old_pid = int(card.process.processId())
        card.restart()
        wait_until(lambda: card.process.state() == QProcess.Running and int(card.process.processId()) != old_pid)
        card.restart()
        wait_until(lambda: card.pending_restart and card.process.state() == QProcess.NotRunning)
        window.stop_all()
        wait_until(lambda: all(not item.process or item.process.state() == QProcess.NotRunning for item in window.cards))
        deadline = time.monotonic() + 0.4
        while time.monotonic() < deadline:
            app.processEvents()
            time.sleep(0.02)
        assert card.process.state() == QProcess.NotRunning, "A pending restart survived Stop All"
        window.close()
        wait_until(lambda: window.close_allowed)
        print("PASS: owned test process start/restart/stop and clean window close")
        print("Preview: /tmp/brl-framework-gui-preview.png")


if __name__ == "__main__":
    main()
