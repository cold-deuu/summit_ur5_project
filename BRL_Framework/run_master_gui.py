#!/usr/bin/env python3
"""BRL Framework desktop GUI; process management adapted from 01's master GUI."""

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import shlex
import signal
import sys
import time

import yaml
from PyQt5.QtCore import QProcess, QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication, QFileDialog, QGridLayout, QHBoxLayout, QLabel, QLineEdit,
    QMainWindow, QMessageBox, QPushButton, QScrollArea, QTextEdit, QVBoxLayout,
    QWidget, QComboBox, QTabWidget, QTreeWidget, QTreeWidgetItem, QHeaderView,
)

WORKSPACE_ROOT = Path(__file__).resolve().parent
SOURCE_ROOT = WORKSPACE_ROOT / "src"
PLAN_DIRECTORY = SOURCE_ROOT / "task_planner/templates/tomato/offline_plans"
sys.path.insert(0, str(SOURCE_ROOT / "task_planner/src"))
from task_planner.offline_plan import OfflinePlanner, execution_action


class ComponentCard(QWidget):
    def __init__(self, manager, name, command, ready_checks):
        super().__init__()
        self.manager = manager
        self.name = name
        self.command = command
        self.ready_checks = ready_checks
        self.process = None
        self.pending_restart = False
        self.stopping = False
        self.log_handle = None

        self.setStyleSheet("ComponentCard { border: 1px solid #555; border-radius: 5px; }")
        layout = QVBoxLayout(self)
        header = QHBoxLayout()
        self.title = QLabel(name)
        self.title.setStyleSheet("font-weight: bold; font-size: 14px;")
        self.status = QLabel("STOPPED")
        self.status.setAlignment(Qt.AlignCenter)
        self.status.setMinimumWidth(120)
        self.restart_button = QPushButton("Restart")
        self.stop_button = QPushButton("Stop")
        self.restart_button.clicked.connect(self.restart)
        self.stop_button.clicked.connect(self.stop)
        header.addWidget(self.title)
        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.start)
        header.addWidget(self.start_button)
        header.addStretch()
        header.addWidget(self.status)
        header.addWidget(self.restart_button)
        header.addWidget(self.stop_button)
        layout.addLayout(header)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setLineWrapMode(QTextEdit.NoWrap)
        self.log.setMinimumHeight(150)
        self.log.document().setMaximumBlockCount(1000)
        layout.addWidget(self.log)
        self._set_status("STOPPED", "#777")

    def _set_status(self, text, color):
        self.status.setText(text)
        self.status.setStyleSheet(
            "color: white; background: %s; padding: 4px; border-radius: 3px;" % color
        )

    def append(self, text):
        if not text:
            return
        cursor = self.log.textCursor()
        cursor.movePosition(cursor.End)
        cursor.insertText(text)
        self.log.setTextCursor(cursor)
        self.log.ensureCursorVisible()
        if self.log_handle:
            self.log_handle.write(text)
            self.log_handle.flush()

    def start(self):
        if self.manager.close_requested:
            return
        if self.process and self.process.state() != QProcess.NotRunning:
            return
        if not self.manager.validate_setup():
            return
        self.manager.stopping_all = False
        self.manager.ensure_log_run()
        self.command = self.manager.component_command(self.name)
        self.pending_restart = False
        self.stopping = False
        self.log.clear()
        log_path = self.manager.log_dir / (self.name + ".log")
        if self.log_handle:
            self.log_handle.close()
        self.log_handle = log_path.open("a", encoding="utf-8")
        self.append("$ %s\n" % shlex.join(self.command))

        shell_command = (
            "source %s && export PYTHONUNBUFFERED=1 "
            "&& exec stdbuf -oL -eL %s"
        ) % (
            shlex.quote(str(self.manager.setup_file)),
            shlex.join(self.command),
        )
        self.process = QProcess(self)
        self.process.setWorkingDirectory(str(WORKSPACE_ROOT))
        self.process.setProcessChannelMode(QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._read_output)
        self.process.started.connect(
            lambda: self._set_status("PROCESS RUNNING", "#27833d")
        )
        self.process.errorOccurred.connect(self._process_error)
        self.process.finished.connect(self._finished)
        self._set_status("STARTING", "#a56a00")
        self.process.start("setsid", ["bash", "-lc", shell_command])

    def _read_output(self):
        data = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        self.append(data)

    def _process_error(self, error):
        if error == QProcess.FailedToStart and self.log_handle:
            self.log_handle.close()
            self.log_handle = None
        if self.stopping or self.pending_restart or self.manager.stopping_all:
            return
        self.append("\n[GUI] Process error: %s\n" % self.process.errorString())
        self._set_status("ERROR", "#a32222")

    def _finished(self, exit_code, exit_status):
        self._read_output()
        if self.pending_restart:
            self.append("\n[GUI] Restarting...\n")
            QTimer.singleShot(300, self._restart_if_requested)
            return
        if self.manager.stopping_all or self.stopping:
            self._set_status("STOPPED", "#777")
        elif exit_code == 0:
            self._set_status("EXITED (0)", "#777")
        else:
            self._set_status("FAILED (%d)" % exit_code, "#a32222")
            self.append("\n[GUI] Unexpected exit code: %d\n" % exit_code)
        if self.log_handle:
            self.log_handle.close()
            self.log_handle = None

    def restart(self):
        if self.manager.close_requested:
            return
        if self.process and self.process.state() != QProcess.NotRunning:
            self.pending_restart = True
            self.stop()
        else:
            self.start()

    def _restart_if_requested(self):
        if self.pending_restart and not self.manager.stopping_all and not self.manager.close_requested:
            self.start()

    def stop(self):
        if not self.process or self.process.state() == QProcess.NotRunning:
            if self.pending_restart:
                QTimer.singleShot(0, self._restart_if_requested)
            else:
                self._set_status("STOPPED", "#777")
            return
        self.stopping = True
        self._set_status("STOPPING", "#a56a00")
        target_process = self.process
        target_pid = int(target_process.processId())
        try:
            if target_pid > 0:
                os.killpg(target_pid, signal.SIGINT)
            else:
                target_process.terminate()
        except (ProcessLookupError, PermissionError):
            target_process.terminate()
        QTimer.singleShot(5000, lambda: self._force_stop(target_process, target_pid))

    def _force_stop(self, target_process, target_pid):
        if target_process.state() != QProcess.NotRunning:
            if self.process is target_process:
                self.append("\n[GUI] SIGINT timeout; sending SIGTERM\n")
            try:
                if target_pid > 0:
                    os.killpg(target_pid, signal.SIGTERM)
                else:
                    target_process.terminate()
            except (ProcessLookupError, PermissionError):
                target_process.kill()
            QTimer.singleShot(2000, lambda: self._kill_stop(target_process, target_pid))

    def _kill_stop(self, target_process, target_pid):
        if target_process.state() != QProcess.NotRunning:
            if self.process is target_process:
                self.append("\n[GUI] SIGTERM timeout; sending SIGKILL\n")
            try:
                if target_pid > 0:
                    os.killpg(target_pid, signal.SIGKILL)
                else:
                    target_process.kill()
            except (ProcessLookupError, PermissionError):
                target_process.kill()

class MasterWindow(QMainWindow):
    def component_command(self, name):
        return next(command for key, command, _ in self._component_commands() if key == name)

    def validate_setup(self):
        if self.setup_file.is_file():
            return True
        QMessageBox.warning(self, "Workspace setup missing",
                            "Build the workspace or pass --setup PATH:\n%s" % self.setup_file)
        return False

    def ensure_log_run(self):
        if self.log_dir is None:
            stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f_gui")
            self.log_dir = self.log_root / stamp
            self.log_dir.mkdir(parents=True, exist_ok=True)
            self.log_label.setText("Logs: %s" % self.log_dir)

    def _shell(self, command):
        return "source %s && export PYTHONUNBUFFERED=1 && exec %s" % (
            shlex.quote(str(self.setup_file)), shlex.join(command))

    def run_command(self, command, callback=None, timeout=30000, quiet=False):
        """Run service calls and readiness probes without blocking the Qt event loop."""
        if self.close_requested or not self.validate_setup():
            return
        self.ensure_log_run()
        process = QProcess(self)
        process.setWorkingDirectory(str(WORKSPACE_ROOT))
        process.setProcessChannelMode(QProcess.MergedChannels)
        self.jobs.add(process)
        timer = QTimer(process)
        timer.setSingleShot(True)
        completed = False

        def finish(code):
            nonlocal completed
            if completed:
                return
            completed = True
            timer.stop()
            output = bytes(process.readAllStandardOutput()).decode("utf-8", errors="replace")
            self.jobs.discard(process)
            if not quiet:
                message = "$ %s\n%s\n[exit %s]\n" % (shlex.join(command), output, code)
                self.control_log.insertPlainText(message)
                with (self.log_dir / "controls.log").open("a", encoding="utf-8") as handle:
                    handle.write(message)
            if callback and not self.close_requested:
                callback(code == 0, output)
            process.deleteLater()

        process.finished.connect(lambda code, _status: finish(code))
        process.errorOccurred.connect(
            lambda error: finish(-1) if error == QProcess.FailedToStart else None)

        def timed_out():
            self._stop_job(process, signal.SIGKILL)

        timer.timeout.connect(timed_out)
        process.start("setsid", ["bash", "-lc", self._shell(command)])
        timer.start(timeout)

    @staticmethod
    def _stop_job(process, sig):
        if process.state() != QProcess.NotRunning:
            pid = int(process.processId())
            if pid > 0:
                try:
                    os.killpg(pid, sig)
                    return
                except (ProcessLookupError, PermissionError):
                    pass
            process.kill()

    def start_all(self):
        if self.close_requested or self.starting_all or not self.validate_setup():
            return
        self.stopping_all = False
        self.starting_all = True
        self.generation += 1
        self.ensure_log_run()
        self._start_next(0, self.generation)

    def _start_next(self, index, generation):
        if generation != self.generation or self.stopping_all:
            return
        if index == len(self.cards):
            self.starting_all = False
            return
        card = self.cards[index]
        card.start()
        deadline = time.monotonic() + self.args.wait_timeout
        QTimer.singleShot(500, lambda: self._probe(index, generation, deadline, 0))

    def _probe(self, index, generation, deadline, check_index):
        if generation != self.generation or self.stopping_all:
            return
        card = self.cards[index]
        if not card.process or card.process.state() == QProcess.NotRunning:
            card.append("\n[GUI] Process exited before becoming ready\n")
            self._start_next(index + 1, generation)
        elif check_index == len(card.ready_checks):
            card._set_status("READY" if card.ready_checks else "PROCESS RUNNING", "#27833d")
            self._start_next(index + 1, generation)
        elif time.monotonic() >= deadline:
            card._set_status("NOT READY", "#a32222")
            card.append("\n[GUI] Readiness timeout; continuing to the next component\n")
            self._start_next(index + 1, generation)
        else:
            def checked(ok, _output):
                if generation != self.generation or self.stopping_all:
                    return
                if ok:
                    self._probe(index, generation, deadline, check_index + 1)
                else:
                    QTimer.singleShot(500, lambda: self._probe(index, generation, deadline, 0))
            self.run_command(card.ready_checks[check_index], checked, timeout=2000, quiet=True)

    def stop_all(self):
        self.stopping_all = True
        self.starting_all = False
        self.generation += 1
        for process in list(self.jobs):
            self._stop_job(process, signal.SIGKILL)
        for card in reversed(self.cards):
            card.pending_restart = False
            card.stop()

    def restart_all(self):
        self.stop_all()
        generation = self.generation

        def restart_when_stopped():
            if self.close_requested or generation != self.generation:
                return
            if self.jobs or any(c.process and c.process.state() != QProcess.NotRunning for c in self.cards):
                QTimer.singleShot(200, restart_when_stopped)
                return
            self.log_dir = None
            self.start_all()
        restart_when_stopped()

    def _close_when_stopped(self):
        if (self.jobs or any(c.process and c.process.state() != QProcess.NotRunning for c in self.cards)
                or (self.monitor and self.monitor.state() != QProcess.NotRunning)):
            QTimer.singleShot(200, self._close_when_stopped)
            return
        self.close_allowed = True
        self.close()

    def __init__(self, args):
        super().__init__()
        self.args = args
        self.setup_file = Path(args.setup).expanduser().resolve()
        self.log_root = Path(args.log_dir).expanduser().resolve()
        self.log_dir = None
        self.cards, self.jobs = [], set()
        self.stopping_all = self.close_requested = self.close_allowed = False
        self.starting_all = False
        self.generation = 0
        self.connected = self.planner_ready = self.control_busy = False
        self.runtime_status = "idle"
        self.status_version = 0
        self.pause_requested = False
        self.preview_actions = []
        self.current_row = -1
        self.monitor = None
        self.monitor_buffer = ""

        # Match the native Qt look and compact header / two-column log cards
        # used by 01, 03, and 04. Runtime details are optional, below the controls.
        self.setWindowTitle("BRL Framework Master Manager")
        self.resize(1450, 900)
        root = QWidget()
        layout = QVBoxLayout(root)
        controls = QHBoxLayout()
        self.log_label = QLabel("Logs: created when a component starts")
        self.log_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        controls.addWidget(self.log_label)
        controls.addStretch()
        self.plan_combo = QComboBox()
        self.plan_combo.addItems([path.stem for path in sorted(PLAN_DIRECTORY.glob("*.yaml"))] + ["사용자 지정"])
        controls.addWidget(QLabel("scenario:"))
        controls.addWidget(self.plan_combo)
        for title, callback in (("Start All", self.start_all), ("Restart All", self.restart_all), ("Stop All", self.stop_all)):
            button = QPushButton(title)
            button.clicked.connect(callback)
            controls.addWidget(button)
        layout.addLayout(controls)

        planning = QHBoxLayout()
        self.plan_path = QLineEdit()
        self.plan_path.setPlaceholderText("Offline plan YAML path")
        browse = QPushButton("Browse YAML")
        browse.clicked.connect(self.browse_plan)
        self.load_button = QPushButton("Load Plan")
        self.load_button.clicked.connect(lambda: self.load_plan())
        self.start_button = QPushButton("Run")
        self.start_button.clicked.connect(self.start_plan)
        self.pause_button = QPushButton("Pause")
        self.pause_button.clicked.connect(lambda: self.control("pause"))
        self.resume_button = QPushButton("Resume")
        self.resume_button.clicked.connect(lambda: self.control("resume"))
        self.stop_button = QPushButton("Stop Plan")
        self.stop_button.clicked.connect(lambda: self.control("stop"))
        planning.addWidget(self.plan_path, 1)
        for button in (browse, self.load_button, self.start_button, self.pause_button, self.resume_button, self.stop_button):
            planning.addWidget(button)
        layout.addLayout(planning)

        status = QHBoxLayout()
        self.status_label = QLabel("상태: 대기")
        self.action_label = QLabel("현재 액션: —")
        self.action_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.connection_label = QLabel("ROS 연결 대기")
        status.addWidget(self.status_label)
        status.addWidget(self.action_label, 1)
        status.addWidget(self.connection_label)
        status.addWidget(QLabel("Robot: %s (%s)" % (args.robot_name, args.robot_type)))
        layout.addLayout(status)
        self.target_label = QLabel("Detect: 카메라와 무관하게 로그의 tomato1 / tomato2 고정 좌표를 제공합니다.")
        self.target_label.setWordWrap(True)
        self.target_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.target_label)
        info = QHBoxLayout()
        self.plan_label = QLabel()
        info.addWidget(self.plan_label, 1)
        self.details_button = QPushButton("Details")
        self.details_button.setCheckable(True)
        info.addWidget(self.details_button)
        layout.addLayout(info)

        self.details_panel = QTabWidget()
        self.details_panel.setMaximumHeight(220)
        self.plan_tree = QTreeWidget()
        self.plan_tree.setHeaderLabels(["Order", "Index", "Offline action"])
        self.plan_tree.setRootIsDecorated(False)
        self.plan_tree.header().setSectionResizeMode(2, QHeaderView.Stretch)
        self.plan_tree.setColumnWidth(0, 55)
        self.plan_tree.setColumnWidth(1, 55)
        self.details_panel.addTab(self.plan_tree, "Plan")
        scene_panel = QWidget()
        scene_layout = QVBoxLayout(scene_panel)
        self.scene_label = QLabel("환경: /scene_graph 대기")
        scene_layout.addWidget(self.scene_label)
        self.scene_tree = QTreeWidget()
        self.scene_tree.setRootIsDecorated(False)
        self.scene_tree.setHeaderLabels(["로봇 / 객체", "상태", "위치 / 보유 로봇", "분류 / 보유 객체"])
        self.scene_tree.header().setSectionResizeMode(QHeaderView.ResizeToContents)
        scene_layout.addWidget(self.scene_tree)
        self.details_panel.addTab(scene_panel, "Scene")
        self.details_panel.hide()
        self.details_button.toggled.connect(self.details_panel.setVisible)
        self.details_button.toggled.connect(lambda checked: self.details_button.setText("Hide Details" if checked else "Details"))
        layout.addWidget(self.details_panel)

        content = QWidget()
        grid = QGridLayout(content)
        for index, (name, command, checks) in enumerate(self._component_commands()):
            card = ComponentCard(self, name, command, checks)
            if name == "motion_planning":
                card.start_button.setToolTip("01 모션 노드의 초기화 및 그리퍼 초기 열기를 실행합니다.")
            self.cards.append(card)
            grid.addWidget(card, index // 2, index % 2)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(content)
        layout.addWidget(scroll, 1)
        self.control_log = QTextEdit()
        self.control_log.setReadOnly(True)
        self.control_log.setMaximumHeight(130)
        self.control_log.document().setMaximumBlockCount(500)
        layout.addWidget(self.control_log)
        self.setCentralWidget(root)
        self.plan_path.textChanged.connect(self.preview_plan)
        self.plan_combo.currentTextChanged.connect(self.select_builtin_plan)
        initial = Path(args.plan).expanduser()
        if not initial.suffix:
            initial = initial.with_suffix(".yaml")
        if not initial.is_absolute():
            initial = PLAN_DIRECTORY / initial
        self.plan_path.setText(str(initial))
        self.control_log.append("Start All starts nodes only. Load Plan, then Run to execute.")
        if not args.no_telemetry:
            QTimer.singleShot(0, self.start_monitor)

    def _component_commands(self):
        robot = "_robot_name:=" + self.args.robot_name
        commands = []
        if not self.args.no_state:
            commands.append(("scene_state", ["rosrun", "knowledge_interface", "knowledge_graph_node.py",
                "_environment_config:=" + str(Path(self.args.environment_config).expanduser().resolve())],
                [["rosservice", "info", "/knowledge_graph_node/robot/update"]]))
        if not self.args.no_sensor:
            commands.append(("brl_sensor", ["rosrun", "brl_sensor", "sensor_node.py", robot,
                "_robot_type:=" + self.args.robot_type],
                [["rosservice", "info", "/%s/trigger_sensor_scan" % self.args.robot_name]]))
        if not self.args.no_motion:
            commands.append(("motion_planning", ["rosrun", "motion_planning", "motion_planner_node.py", robot],
                [["rostopic", "info", "/%s/%s/status" % (self.args.robot_name, action)]
                 for action in ("pick_tomato", "scan_tomato", "place_tomato", "prepare_navigation")]))
        if not self.args.no_task_planner:
            commands.append(("task_planner", ["rosrun", "task_planner", "task_planner_node.py", robot,
                "_offline_plan:=" + self.plan_path.text().strip()],
                [["rosservice", "info", "/generate_task_plan"], ["rosservice", "info", "/task_planner/start"]]))
        return commands

    def browse_plan(self):
        path, _ = QFileDialog.getOpenFileName(self, "Offline plan 선택", str(PLAN_DIRECTORY), "YAML (*.yaml *.yml)")
        if path:
            self.plan_path.setText(path)

    def select_builtin_plan(self, name):
        if name == "사용자 지정":
            self.browse_plan()
            # Restore the selection if the file dialog was cancelled, or if the
            # selected path was already in the input and emitted no change.
            self.preview_plan()
        else:
            self.plan_path.setText(str(PLAN_DIRECTORY / (name + ".yaml")))

    def preview_plan(self):
        path = Path(self.plan_path.text()).expanduser()
        builtin = path.stem if path.resolve().parent == PLAN_DIRECTORY.resolve() else "사용자 지정"
        if self.plan_combo.findText(builtin) < 0:
            builtin = "사용자 지정"
        self.plan_combo.blockSignals(True)
        self.plan_combo.setCurrentText(builtin)
        self.plan_combo.blockSignals(False)
        self.plan_tree.clear()
        self.preview_actions = []
        self.current_row = -1
        try:
            planner = OfflinePlanner(Path(self.plan_path.text()).expanduser())
            for order, entry in enumerate(planner.offline_plan, 1):
                action = execution_action(entry["name"])
                if action.arguments[0] != self.args.robot_name:
                    raise ValueError("Plan의 로봇 이름이 %s과 다릅니다." % self.args.robot_name)
                self.preview_actions.append(action.name)
                self.plan_tree.addTopLevelItem(QTreeWidgetItem([str(order), str(entry["index"]), action.name]))
            self.plan_label.setText("%d개 액션 · 파일 순서대로 실행 · base 이동 없음" % len(self.preview_actions))
            self.plan_label.setStyleSheet("color: #27833d;")
        except (OSError, ValueError, yaml.YAMLError) as error:
            self.preview_actions = []
            self.plan_tree.clear()
            self.plan_label.setText("Plan 확인: " + str(error))
            self.plan_label.setStyleSheet("color: #a32222;")
        self.update_buttons()

    def update_buttons(self):
        ready = self.connected and self.planner_ready
        running = self.runtime_status == "running"
        editing = not running and not self.control_busy
        self.load_button.setEnabled(ready and editing and bool(self.preview_actions))
        self.start_button.setEnabled(ready and editing and bool(self.preview_actions))
        self.pause_button.setEnabled(ready and running and not self.pause_requested and not self.control_busy)
        self.resume_button.setEnabled(ready and running and self.pause_requested and not self.control_busy)
        # Keep cancellation available even while another command is pending.
        self.stop_button.setEnabled(ready and running)

    def service_call(self, name, payload, callback):
        command = ["rosservice", "call", name, json.dumps(payload, ensure_ascii=False)]
        generation = self.generation
        def finished(ok, output):
            try:
                response = yaml.safe_load(output)
                accepted = (generation == self.generation and ok and
                            isinstance(response, dict) and response.get("success") is True)
                message = response.get("message", output) if isinstance(response, dict) else output
            except yaml.YAMLError:
                accepted, message = False, output
            self.control_log.append(("[OK] " if accepted else "[FAILED] ") + name + ": " + str(message))
            callback(accepted, message)
        self.run_command(command, finished, timeout=15000)

    def load_plan(self, start_after=False):
        # clicked(bool) is not an instruction to auto-start after loading.
        if self.control_busy or self.close_requested or not self.validate_setup():
            return
        self.preview_plan()
        if not self.preview_actions:
            return
        self.control_busy = True
        self.update_buttons()
        payload = {"instruction": str(Path(self.plan_path.text()).expanduser().resolve()), "preferences_json": ""}
        start_status_version = None
        def loaded(ok, message):
            nonlocal start_status_version
            if ok and start_after:
                start_status_version = self.status_version
                self.current_row = -1
                self.pause_requested = False
                self.service_call("/task_planner/start", {}, started)
            else:
                self.control_busy = False
                self.status_label.setText("상태: " + ("Plan 불러오기 완료" if ok else "불러오기 실패"))
                self.update_buttons()
        def started(ok, message):
            self.control_busy = False
            # A placeholder-only plan can finish before the start service reply.
            # Preserve any newer ROS status rather than overwriting completion.
            if ok and self.status_version == start_status_version:
                self.runtime_status = "running"
                self.status_label.setText("상태: 실행 중")
            elif not ok:
                self.status_label.setText("상태: 시작 실패")
            self.update_buttons()
        self.service_call("/generate_task_plan", payload, loaded)

    def start_plan(self):
        # Always prepare the selected file first, including for external nodes.
        self.load_plan(start_after=True)

    def control(self, action):
        if self.close_requested or not self.validate_setup():
            return
        if action != "stop":
            self.control_busy = True
            self.update_buttons()
        def completed(ok, message):
            self.control_busy = False
            if ok:
                if action == "pause":
                    self.pause_requested = True
                    self.status_label.setText("상태: 일시정지 요청 · 현재 액션 후 적용")
                elif action == "resume":
                    self.pause_requested = False
                    self.status_label.setText("상태: 실행 중")
                else:
                    self.status_label.setText("상태: 중지 요청 · 액션 취소 대기")
            self.update_buttons()
        self.service_call("/task_planner/" + action, {}, completed)

    def start_monitor(self):
        if self.close_requested or not self.setup_file.is_file():
            self.connection_label.setText("ROS 상태: workspace 빌드 필요")
            return
        process = QProcess(self)
        self.monitor = process
        self.monitor_buffer = ""
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(self.read_monitor)
        process.finished.connect(lambda *_: QTimer.singleShot(2000, self.start_monitor) if not self.close_requested else None)
        process.errorOccurred.connect(lambda _: self.connection_label.setText("ROS 상태 수신기 실행 실패"))
        command = [sys.executable, str(WORKSPACE_ROOT / "tools/gui_ros_bridge.py"), "--robot-name", self.args.robot_name]
        process.start("setsid", ["bash", "-lc", self._shell(command)])

    def read_monitor(self):
        self.monitor_buffer += bytes(self.monitor.readAllStandardOutput()).decode("utf-8", errors="replace")
        while "\n" in self.monitor_buffer:
            line, self.monitor_buffer = self.monitor_buffer.split("\n", 1)
            if line.startswith("BRL_GUI:"):
                try:
                    self.receive_event(json.loads(line[len("BRL_GUI:"):]))
                except (ValueError, KeyError, TypeError):
                    continue

    def receive_event(self, event):
        if event["type"] == "connection":
            self.connected = event["connected"]
            self.planner_ready = event.get("planner_ready", False)
            self.connection_label.setText("ROS 연결됨" if self.connected else "ROS master 연결 대기")
            self.connection_label.setStyleSheet("color: " + ("#27833d;" if self.connected else "#a56a00;"))
        elif event["type"] == "status":
            self.status_version += 1
            status = event["status"]
            if status == "running" and self.runtime_status != "running":
                self.current_row = -1
            self.runtime_status = status
            if status != "running":
                self.pause_requested = False
            labels = {"idle": "대기", "running": "실행 중", "completed": "완료", "failed": "실패", "stopped": "중지됨"}
            if not self.pause_requested:
                self.status_label.setText("상태: " + labels.get(status, status))
        elif event["type"] == "action":
            action = event["action"]
            self.action_label.setText("현재 액션: " + (action or "—"))
            if action:
                candidates = list(range(self.current_row + 1, len(self.preview_actions))) + list(range(self.current_row + 1))
                for index in candidates:
                    if self.preview_actions[index].replace(" ", "") == action.replace(" ", ""):
                        self.current_row = index
                        item = self.plan_tree.topLevelItem(index)
                        self.plan_tree.setCurrentItem(item)
                        self.plan_tree.scrollToItem(item)
                        break
        elif event["type"] == "scene":
            self.scene_label.setText("환경: %s · 로봇 %d · 객체 %d" % (event["environment"], len(event["robots"]), len(event["objects"])))
            self.scene_tree.clear()
            for robot in event["robots"]:
                self.scene_tree.addTopLevelItem(QTreeWidgetItem([robot["name"], robot["status"], robot["location"], ", ".join(robot["held_objects"])]))
            for obj in event["objects"]:
                self.scene_tree.addTopLevelItem(QTreeWidgetItem([obj["name"], obj["status"], obj["holder"] or obj["location"], obj["class"]]))
        elif event["type"] == "targets":
            targets = event["targets"]
            labels = ["%s → %s  xyz=(%.3f, %.3f, %.3f) m  [로그 고정값] → %s" % (
                target["alias"], target["name"], target["x"], target["y"], target["z"],
                target["destination"]) for target in targets]
            self.target_label.setText("Detect 결과: " + ("\n".join(labels) if labels else "로그 고정 좌표 반환 대기"))
        self.update_buttons()

    def closeEvent(self, event):
        if self.close_allowed:
            event.accept()
            return
        event.ignore()
        if self.close_requested:
            return
        self.close_requested = True
        self.setWindowTitle("BRL Framework Master Manager - stopping processes...")
        self.stop_all()
        if self.monitor:
            self._stop_job(self.monitor, signal.SIGINT)
            QTimer.singleShot(3000, lambda: self._stop_job(self.monitor, signal.SIGKILL))
        self._close_when_stopped()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="BRL Framework offline desktop GUI")
    parser.add_argument("--setup", default=str(WORKSPACE_ROOT / "devel/setup.bash"))
    parser.add_argument("--robot-name", default="brl_robot")
    parser.add_argument("--robot-type", default="harvester")
    parser.add_argument("--plan", default="T1", help="initial plan selection; does not execute it")
    parser.add_argument("--environment-config", default=str(SOURCE_ROOT / "knowledge_interface/config/real_farm_config.json"))
    for name in ("state", "sensor", "motion", "task-planner"):
        parser.add_argument("--no-" + name, action="store_true", help="use an external node instead")
    parser.add_argument("--no-telemetry", action="store_true", help="open the window without ROS status subscriptions")
    parser.add_argument("--wait-timeout", type=int, default=30)
    parser.add_argument("--log-dir", default=str(WORKSPACE_ROOT / "logs/master_gui"))
    args = parser.parse_args(argv)
    if args.wait_timeout <= 0:
        parser.error("--wait-timeout must be positive")
    return args


def main():
    args = parse_args()
    app = QApplication(sys.argv[:1])
    window = MasterWindow(args)
    window.show()
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal.signal(signal.SIGTERM, lambda *_: window.close())
    heartbeat = QTimer()
    heartbeat.timeout.connect(lambda: None)
    heartbeat.start(200)
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
