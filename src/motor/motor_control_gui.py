"""PySide6 engineering dashboard for bounded MHF9025-V3 experiments.

Hardware access occurs only in :class:`MotorControlWorker`.  Use ``--simulate``
for a complete software-only demonstration that never opens a serial device.
"""

from __future__ import annotations

import argparse
from collections import deque
import ctypes.util
from datetime import datetime
import math
import os
from pathlib import Path
import queue
import shutil
import sys
import threading
import time

try:
    from PySide6 import QtCore, QtGui, QtWidgets
    import pyqtgraph as pg
except ImportError as exc:  # Friendly error when launched before requirements install.
    raise SystemExit("PySide6 and pyqtgraph are required; install requirements.txt") from exc

from motor_gui_core import (AUTO_HOME_DEFAULT_SPEED_RPM, GUI_CONFIRMATION, GUI_HARD_SPEED_RPM,
                            MAX_GUI_DURATION_S, MAX_GUI_RPM,
                            PID_FIELD_NAMES, MotorSessionController, SimulatedWaveshareMotor,
                            validate_auto_home_request, validate_motion_request)
from motor_can import CommunicationError
from joint_state_publisher import JointStateEstimator, RclpyJointStatePublisher
from waveshare_motor import WaveshareMotor
from absolute_home_calibration import FINAL_CONFIRMATION


def require_qt_platform_runtime() -> None:
    """Fail with an actionable message before Qt aborts on an incomplete X11 runtime."""
    platform = os.environ.get("QT_QPA_PLATFORM", "")
    using_x11 = bool(os.environ.get("DISPLAY")) and platform not in {
        "offscreen", "minimal", "linuxfb", "vnc", "wayland", "wayland-egl"
    }
    if sys.platform.startswith("linux") and using_x11 and ctypes.util.find_library("xcb-cursor") is None:
        raise SystemExit(
            "Missing Ubuntu Qt/X11 runtime library libxcb-cursor.so.0.\n"
            "Install it with: sudo apt install -y libxcb-cursor0\n"
            "Then run: python motor_control_gui.py")


class MotorControlWorker(QtCore.QObject):
    telemetry = QtCore.Signal(dict)
    event = QtCore.Signal(str, str)
    state_changed = QtCore.Signal(str)
    connection_changed = QtCore.Signal(bool, str)
    session_finished = QtCore.Signal(bool, str)
    csv_ready = QtCore.Signal(str)
    authorization_required = QtCore.Signal(str)
    pid_received = QtCore.Signal(dict)
    pid_error = QtCore.Signal(str)
    home_result = QtCore.Signal(dict)
    home_error = QtCore.Signal(str)
    home_authorization_required = QtCore.Signal(dict)
    finished = QtCore.Signal()

    def __init__(self, *, simulate: bool = False, log_dir=None, home_log_dir=None):
        super().__init__()
        self.simulate = simulate
        self.log_dir = log_dir
        self.home_log_dir = (home_log_dir if home_log_dir is not None else
                             Path(__file__).resolve().parent / "simulated_home_logs"
                             if simulate else None)
        self._commands: queue.PriorityQueue = queue.PriorityQueue()
        self._sequence = 0
        self._shutdown = threading.Event()
        self._authorization_ready = threading.Event()
        self._authorization = ""
        self._home_authorization_ready = threading.Event()
        self._home_authorization = ""
        self._controller_lock = threading.Lock()
        self._controller = None

    def _put(self, priority: int, name: str, *args) -> None:
        self._sequence += 1
        self._commands.put((priority, self._sequence, name, args))

    def connect_motor(self, device: str, bitrate: int, motor_id: int) -> None:
        self._put(10, "connect", device, bitrate, motor_id)

    def disconnect_motor(self) -> None:
        self.request_stop("disconnect")
        self._put(1, "disconnect")

    def start_motion(self, rpm: float, duration: float, auto_home: bool = True,
                     home_tolerance_deg: float = 1.0,
                     home_speed_rpm: float = AUTO_HOME_DEFAULT_SPEED_RPM,
                     position_hold: bool = False) -> None:
        self._put(5, "start", rpm, duration, auto_home,
                  home_tolerance_deg, home_speed_rpm, position_hold)

    def request_home_motion(self, tolerance_deg: float, speed_rpm: float,
                            position_hold: bool = False) -> None:
        self._put(5, "home_motion", tolerance_deg, speed_rpm, position_hold)

    def submit_authorization(self, phrase: str) -> None:
        self._authorization = phrase
        self._authorization_ready.set()

    def _wait_for_authorization(self, action: str = "start", *,
                                position_hold: bool = False) -> str:
        self._authorization = ""
        self._authorization_ready.clear()
        descriptions = {
            "start": "START will run at the selected target speed for the selected duration.",
            "start_with_home": (
                "START will first move to the stored absolute HOME, "
                "verify the angle, then run at the selected target speed and duration."),
            "home_only": (
                "GO HOME will move to the stored absolute HOME. "
                "It will not run the selected target speed or duration."),
        }
        self.authorization_required.emit(
            "Preflight passed. Protection remains UNKNOWN. Confirm the motor is secured, "
            "the shaft is unobstructed, and an independent power cutoff is ready.\n\n" +
            descriptions[action] +
            ("\n\nEXPERIMENTAL POSITION HOLD selected: at HOME, switch directly "
             "to V2.36 0xA4 at encoder HOME; after the timed run, reduce speed "
             "to 0.5 RPM, then switch directly to 0xA4 at the fresh 0x92 angle. "
             "0xA4 has a 1 deg/s speed limit. STOP/DISABLE remains the fallback. "
             "V3 behavior and physical holding are unverified. "
             "The controller remains energized until DISABLE or disconnect."
             if position_hold else ""))
        deadline = time.monotonic() + 60
        while not self._authorization_ready.wait(0.05):
            with self._controller_lock:
                controller = self._controller
            if self._shutdown.is_set() or controller is None or controller.stop_requested.is_set():
                return ""
            if time.monotonic() >= deadline:
                return ""
        return self._authorization

    def request_stop(self, reason: str = "operator_stop") -> None:
        # Setting Event is thread-safe and does not wait for the worker event loop.
        with self._controller_lock:
            controller = self._controller
        if controller is not None:
            controller.request_stop(reason)
        self._put(0, "stop", reason)

    def request_disable(self) -> None:
        with self._controller_lock:
            active = self._controller is not None and self._controller.motion_active
        self.request_stop("operator_disable")
        if not active:
            self._put(1, "disable")

    def request_shutdown(self) -> None:
        self._shutdown.set()
        self.request_stop("application_shutdown")
        self._put(-1, "shutdown")

    def request_pid_read(self) -> None:
        self._put(6, "read_pid")

    def request_home_angle_read(self) -> None:
        self._put(6, "read_home_angles")

    def request_set_home(self) -> None:
        self._put(4, "set_home")

    def submit_home_authorization(self, phrase: str) -> None:
        self._home_authorization = phrase
        self._home_authorization_ready.set()

    def _wait_home_authorization(self, summary: dict) -> str:
        self._home_authorization = ""
        self._home_authorization_ready.clear()
        self.home_authorization_required.emit(summary)
        deadline = time.monotonic() + 120
        while not self._home_authorization_ready.wait(0.05):
            if self._shutdown.is_set():
                return ""
            with self._controller_lock:
                controller = self._controller
            if controller is None or controller.stop_requested.is_set():
                return ""
            if time.monotonic() >= deadline:
                return ""
        return self._home_authorization

    @QtCore.Slot()
    def run(self) -> None:
        factory = SimulatedWaveshareMotor if self.simulate else WaveshareMotor
        controller = MotorSessionController(
            transport_factory=factory,
            event_callback=lambda level, message: self.event.emit(level, message),
            telemetry_callback=self.telemetry.emit,
            state_callback=self.state_changed.emit,
            log_dir=self.log_dir,
            home_log_dir=self.home_log_dir,
        )
        with self._controller_lock:
            self._controller = controller
        last_idle_poll = 0.0
        try:
            while not self._shutdown.is_set():
                try:
                    _, _, command, args = self._commands.get(timeout=0.02)
                except queue.Empty:
                    command = None
                    args = ()
                try:
                    if command == "connect":
                        controller.connect(*args)
                        label = f"0x{0x140 + args[2]:03X}" + (" (SIMULATED)" if self.simulate else "")
                        self.connection_changed.emit(True, label)
                        restored = controller.load_saved_home_session()
                        if restored is not None:
                            self.home_result.emit(restored)
                    elif command == "disconnect":
                        controller.disconnect()
                        self.connection_changed.emit(False, "Disconnected")
                    elif command == "start":
                        try:
                            controller.start_motion(
                                args[0], args[1],
                                authorization_callback=lambda: self._wait_for_authorization(
                                    "start_with_home" if args[2] else "start",
                                    position_hold=args[5]),
                                auto_home=args[2], home_tolerance_deg=args[3],
                                home_speed_rpm=args[4], position_hold=args[5])
                        except BaseException as exc:
                            self.session_finished.emit(False, str(exc))
                        else:
                            self.session_finished.emit(True, "Bounded session complete")
                        if controller.last_csv_path is not None:
                            self.csv_ready.emit(str(controller.last_csv_path))
                    elif command == "home_motion":
                        try:
                            controller.home_only(
                                args[0], args[1],
                                authorization_callback=lambda: self._wait_for_authorization(
                                    "home_only", position_hold=args[2]),
                                position_hold=args[2])
                        except BaseException as exc:
                            self.session_finished.emit(False, str(exc))
                        else:
                            self.session_finished.emit(True, "GO HOME complete")
                        if controller.last_csv_path is not None:
                            self.csv_ready.emit(str(controller.last_csv_path))
                    elif command == "stop":
                        if controller.connected and not controller.motion_active:
                            if controller.position_hold_active:
                                controller.stop_position_hold()
                            elif controller.enable_uncertain:
                                controller.disable(args[0])
                    elif command == "disable":
                        if controller.connected and not controller.motion_active:
                            controller.disable("operator_disable")
                    elif command == "read_pid":
                        self.pid_received.emit(controller.read_pid())
                    elif command == "read_home_angles":
                        self.home_result.emit(controller._home_result())
                    elif command == "set_home":
                        if not controller.connected or controller.motion_active:
                            raise CommunicationError(
                                "SET HOME requires a connected, idle motor")
                        if not controller.disable("set_home_preflight"):
                            raise CommunicationError(
                                "SET HOME blocked: STOP/DISABLE or stationary feedback "
                                "was not confirmed; use the independent power cutoff "
                                "if physical stopping is uncertain")
                        result = controller.set_absolute_home(
                            lambda summary: self._wait_home_authorization(summary))
                        result["write_attempted_this_connection"] = bool(
                            (result.get("write") or {}).get("attempted"))
                        self.home_result.emit(result)
                    elif command == "shutdown":
                        break
                except BaseException as exc:
                    self.event.emit("ERROR", f"{type(exc).__name__}: {exc}")
                    if command == "set_home":
                        result = controller._home_result(read_angles=False)
                        if (result.get("write") or {}).get("attempted"):
                            result["write_attempted_this_connection"] = True
                            self.home_result.emit(result)
                    if command == "read_pid":
                        self.pid_error.emit(str(exc))
                    if command in ("read_home_angles", "set_home"):
                        self.home_error.emit(str(exc))
                    if command == "connect":
                        self.connection_changed.emit(False, "COMMUNICATION ERROR")
                if (controller.connected and not controller.motion_active and
                        time.monotonic() - last_idle_poll >= 0.125 and
                        self._commands.empty()):
                    try:
                        controller.poll_idle_once()
                    except BaseException as exc:
                        self.event.emit("ERROR", f"Telemetry communication error: {exc}")
                        try:
                            controller.disconnect()
                        except BaseException as cleanup_exc:
                            self.event.emit("ERROR", f"Connection cleanup failed: {cleanup_exc}")
                        self.connection_changed.emit(False, "COMMUNICATION ERROR")
                    last_idle_poll = time.monotonic()
        finally:
            try:
                if controller.connected:
                    controller.disconnect()
            except BaseException as exc:
                self.event.emit("ERROR", f"Shutdown cleanup unconfirmed: {exc}; hardware cutoff required")
            with self._controller_lock:
                self._controller = None
            self.finished.emit()
            # run() owns this thread's long-running command loop.  Quit it
            # directly so GUI shutdown does not depend on the blocked GUI
            # thread processing a queued QThread.quit() call.
            QtCore.QThread.currentThread().quit()


class MotorControlWindow(QtWidgets.QMainWindow):
    def __init__(self, *, simulate: bool = False, publish_joint_states: bool = False,
                 ros_publisher_factory=RclpyJointStatePublisher):
        super().__init__()
        self.simulate = simulate
        self.ros_joint_publisher = (ros_publisher_factory() if publish_joint_states else None)
        self.joint_estimator = (JointStateEstimator() if publish_joint_states else None)
        self._joint_reference_pending_reconnect = False
        self.connected = False
        self.running = False
        self.last_csv_path = ""
        self.plot_epoch = None
        self.raw_times = deque(maxlen=600); self.plot_raw = deque(maxlen=600)
        self.filtered_times = deque(maxlen=600); self.plot_filtered = deque(maxlen=600)
        self.angle_speed_times = deque(maxlen=600); self.plot_angle_speed = deque(maxlen=600)
        self.target_times = deque(maxlen=600); self.plot_target = deque(maxlen=600)
        self.angle_times = deque(maxlen=600); self.plot_angle = deque(maxlen=600)
        self._latest_ui_telemetry = None
        self._displayed_ui_telemetry = None
        self._last_torque_feedback_mono = None
        self.pid_loaded = False
        self.home_busy = False
        self._home_single_degrees = None
        self._home_multi_degrees = None
        self.setWindowTitle("MHF9025-V3 Motor Control" + (" — SIMULATION" if simulate else ""))
        self.resize(1350, 1000)
        self._build_ui()
        self._start_worker()
        self._refresh_controls()

    def _build_ui(self) -> None:
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        layout = QtWidgets.QVBoxLayout(root)
        top = QtWidgets.QHBoxLayout()
        layout.addLayout(top)

        connection = QtWidgets.QGroupBox("Connection")
        form = QtWidgets.QFormLayout(connection)
        self.device = QtWidgets.QLineEdit("SIMULATED" if self.simulate else "/dev/ttyUSB0")
        self.bitrate = QtWidgets.QComboBox()
        self.bitrate.addItem("500000 bit/s", 500_000)
        self.bitrate.addItem("1000000 bit/s", 1_000_000)
        self.bitrate.setCurrentIndex(1)  # Current motor responded at 1 Mbit/s.
        self.motor_id = QtWidgets.QSpinBox(); self.motor_id.setRange(1, 32); self.motor_id.setValue(1)
        self.connection_label = QtWidgets.QLabel("DISCONNECTED")
        self.connection_label.setStyleSheet("font-weight:bold;color:#a33")
        self.can_id_label = QtWidgets.QLabel("—")
        self.connect_button = QtWidgets.QPushButton("CONNECT")
        self.disconnect_button = QtWidgets.QPushButton("DISCONNECT")
        buttons = QtWidgets.QHBoxLayout(); buttons.addWidget(self.connect_button); buttons.addWidget(self.disconnect_button)
        form.addRow("Serial device", self.device); form.addRow("CAN bitrate", self.bitrate)
        form.addRow("Motor ID", self.motor_id); form.addRow("CAN ID", self.can_id_label)
        form.addRow("State", self.connection_label); form.addRow(buttons)
        top.addWidget(connection)

        control = QtWidgets.QGroupBox("Bounded motor control")
        cform = QtWidgets.QFormLayout(control)
        self.speed = QtWidgets.QDoubleSpinBox(); self.speed.setRange(-MAX_GUI_RPM, MAX_GUI_RPM)
        self.speed.setDecimals(2); self.speed.setValue(5.0); self.speed.setSuffix(" RPM")
        self.duration = QtWidgets.QDoubleSpinBox(); self.duration.setRange(0.1, MAX_GUI_DURATION_S)
        self.duration.setDecimals(1); self.duration.setValue(1.0); self.duration.setSuffix(" s")
        self.auto_home = QtWidgets.QCheckBox("Auto HOME before START")
        self.auto_home.setChecked(True)
        self.position_hold = QtWidgets.QCheckBox(
            "Experimental position hold after GO HOME / START")
        self.position_hold.setChecked(False)
        self.position_hold.setToolTip(
            "Opt in to direct 0xA4 position control at HOME and after a bounded "
            "low-speed approach when the timed run ends. "
            "V3 support and holding torque are unverified; keep power cutoff ready.")
        self.auto_home_speed = QtWidgets.QDoubleSpinBox()
        self.auto_home_speed.setRange(0.5, 5.0)
        self.auto_home_speed.setDecimals(1)
        self.auto_home_speed.setValue(AUTO_HOME_DEFAULT_SPEED_RPM)
        self.auto_home_speed.setSuffix(" RPM")
        self.auto_home_speed.setToolTip(
            "Positive-direction coarse approach speed. The final 10 degrees use 0.5 RPM.")
        self.start_button = QtWidgets.QPushButton("START")
        self.go_home_button = QtWidgets.QPushButton("GO HOME")
        self.go_home_button.setToolTip(
            "Move to stored encoder HOME using the approach speed and HOME tolerance; "
            "do not run the target speed or duration.")
        self.stop_button = QtWidgets.QPushButton("STOP")
        self.disable_button = QtWidgets.QPushButton("DISABLE")
        self.stop_button.setToolTip(
            "Normal STOP (0x81) only: does not send DISABLE, so an enabled "
            "controller remains enabled after confirmed stop. "
            "Use DISABLE to turn it off; holding torque is not verified.")
        self.disable_button.setToolTip("Send STOP (0x81) followed by DISABLE (0x80).")
        self.start_button.setStyleSheet("background:#287a3d;color:white;font-weight:bold")
        self.stop_button.setStyleSheet("background:#b21f2d;color:white;font-weight:bold;font-size:16px")
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.start_button)
        row.addWidget(self.go_home_button)
        row.addWidget(self.stop_button)
        row.addWidget(self.disable_button)
        note = QtWidgets.QLabel("Hardware enabled: positive direction only, ≤100 RPM, ≤20 s. Protection: UNKNOWN")
        note.setWordWrap(True); note.setStyleSheet("color:#b06000;font-weight:bold")
        cform.addRow("Target speed", self.speed); cform.addRow("Duration", self.duration)
        cform.addRow(self.auto_home)
        cform.addRow(self.position_hold)
        cform.addRow("HOME approach speed", self.auto_home_speed)
        cform.addRow(row); cform.addRow(note)
        top.addWidget(control)

        pid_box = QtWidgets.QGroupBox("PID parameters — read only, provisional V2.35 raw bytes")
        pid_grid = QtWidgets.QGridLayout(pid_box)
        labels = {
            "angle_kp": "Angle Kp", "angle_ki": "Angle Ki",
            "speed_kp": "Speed Kp", "speed_ki": "Speed Ki",
            "torque_kp": "Torque Kp", "torque_ki": "Torque Ki",
        }
        self.pid_inputs = {}
        for column, name in enumerate(PID_FIELD_NAMES):
            pid_grid.addWidget(QtWidgets.QLabel(labels[name]), 0, column)
            editor = QtWidgets.QSpinBox(); editor.setRange(0, 255); editor.setReadOnly(True)
            editor.setButtonSymbols(QtWidgets.QAbstractSpinBox.ButtonSymbols.NoButtons)
            pid_grid.addWidget(editor, 1, column)
            self.pid_inputs[name] = editor
        self.pid_read_button = QtWidgets.QPushButton("READ PID (0x30)")
        self.pid_status = QtWidgets.QLabel(
            "0x30 read only. Previous MHF9025-V3 attempts timed out. Raw meanings are unverified; "
            "this GUI cannot send PID write commands 0x31 or 0x32.")
        self.pid_status.setWordWrap(True)
        self.pid_status.setStyleSheet("color:#b06000")
        pid_grid.addWidget(self.pid_read_button, 2, 0, 1, 2)
        pid_grid.addWidget(self.pid_status, 2, 2, 1, 4)
        layout.addWidget(pid_box)

        home_box = QtWidgets.QGroupBox("Absolute HOME Calibration")
        home_grid = QtWidgets.QGridLayout(home_box)
        self.home_single = QtWidgets.QLabel("— deg")
        self.home_multi = QtWidgets.QLabel("— deg")
        self.home_theta_deg = QtWidgets.QLabel("— deg")
        self.home_theta_rad = QtWidgets.QLabel("— rad")
        self.home_theta_source = QtWidgets.QLabel("0x94 after startup; 0x92 while running")
        for label in (self.home_single, self.home_multi, self.home_theta_deg,
                      self.home_theta_rad, self.home_theta_source):
            label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.home_direction_sign = QtWidgets.QComboBox()
        self.home_direction_sign.addItem("+1 (observed mounting)", 1)
        self.home_direction_sign.addItem("-1", -1)
        self.home_tolerance = QtWidgets.QDoubleSpinBox()
        self.home_tolerance.setRange(0.01, 5.0)
        self.home_tolerance.setDecimals(2)
        self.home_tolerance.setValue(1.0)
        self.home_tolerance.setSuffix(" deg")
        self.home_status = QtWidgets.QLabel(
            "NOT CALIBRATED IN THIS GUI SESSION — zero angle alone is not verification")
        self.home_status.setWordWrap(True)
        self.home_status.setStyleSheet("color:#b06000;font-weight:bold")
        self.home_read_button = QtWidgets.QPushButton("READ ANGLE")
        self.home_go_button = QtWidgets.QPushButton("GO HOME")
        self.home_go_button.setToolTip(
            "Move to the stored encoder zero; does not write HOME to ROM.")
        self.home_set_button = QtWidgets.QPushButton("SET HOME (ROM WRITE)")
        self.home_set_button.setToolTip(
            "Redefine the stored encoder zero at the current shaft position. "
            "First confirms STOP/DISABLE and stationary feedback, then requires "
            "explicit confirmation before the ROM write.")
        self.home_set_button.setStyleSheet("background:#8a4b08;color:white;font-weight:bold")
        home_buttons = QtWidgets.QHBoxLayout()
        home_buttons.addWidget(self.home_read_button)
        home_buttons.addWidget(self.home_go_button)
        home_buttons.addWidget(self.home_set_button)
        home_grid.addWidget(QtWidgets.QLabel("Absolute single-turn (0x94)"), 0, 0)
        home_grid.addWidget(self.home_single, 0, 1)
        home_grid.addWidget(QtWidgets.QLabel("Accumulated angle (0x92)"), 0, 2)
        home_grid.addWidget(self.home_multi, 0, 3)
        home_grid.addWidget(QtWidgets.QLabel("theta1"), 1, 0)
        home_grid.addWidget(self.home_theta_deg, 1, 1)
        home_grid.addWidget(self.home_theta_rad, 1, 2)
        home_grid.addWidget(self.home_theta_source, 1, 3)
        home_grid.addWidget(QtWidgets.QLabel("direction_sign"), 2, 0)
        home_grid.addWidget(self.home_direction_sign, 2, 1)
        home_grid.addWidget(QtWidgets.QLabel("GO HOME angle tolerance"), 2, 2)
        home_grid.addWidget(self.home_tolerance, 2, 3)
        home_grid.addLayout(home_buttons, 3, 0, 1, 2)
        home_grid.addWidget(self.home_status, 3, 2, 1, 2)
        layout.addWidget(home_box)

        telemetry_box = QtWidgets.QGroupBox("Real-time telemetry — V2.35 interpretation provisional for MHF9025-V3")
        grid = QtWidgets.QGridLayout(telemetry_box)
        names = [
            ("target", "Target speed", "RPM"), ("actual", "Raw reported speed", "RPM"),
            ("filtered", "Filtered display speed", "RPM"),
            ("angle_speed", "Angle-derived average", "RPM"),
            ("tracking_error", "Raw tracking error", "RPM"),
            ("angle_deg", "Accumulated angle", "deg"), ("angle_rad", "Accumulated angle", "rad"),
            ("raw_speed", "Raw motor speed", "deg/s"),
            ("torque_current", "Torque-current feedback (0x9C)", "raw counts"),
            ("torque_age", "Torque-current sample age", "s"),
            ("encoder", "Raw encoder", "counts"),
            ("temperature", "Temperature", "°C"), ("voltage", "Supply voltage", "V"),
            ("motor_state", "Motor state", ""), ("protection", "Protection", ""),
            ("last_response", "Last response UTC", ""), ("feedback_age", "Feedback age", "s"),
            ("speed_hz", "Speed sample rate", "Hz"), ("angle_hz", "Angle sample rate", "Hz"),
            ("communication", "Communication", ""),
            ("stop_confirmation", "Stop confirmation", ""),
        ]
        self.values = {}
        for index, (key, title, unit) in enumerate(names):
            label = QtWidgets.QLabel(title)
            value = QtWidgets.QLabel("—" + (f" {unit}" if unit else ""))
            value.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
            # Live status strings can be much longer than the available cell.
            # Keep their full text selectable without letting sizeHint widen
            # the top-level window after START or STOP.
            value.setMinimumWidth(0)
            value.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
                                QtWidgets.QSizePolicy.Policy.Preferred)
            row, col = divmod(index, 3)
            grid.addWidget(label, row, col * 2); grid.addWidget(value, row, col * 2 + 1)
            self.values[key] = (value, unit)
        torque_note = QtWidgets.QLabel(
            "0x9C torque-current: signed raw counts. A and N·m conversion and "
            "safe V3 limits are unknown. This display does not change torque.")
        torque_note.setWordWrap(True)
        grid.addWidget(torque_note, (len(names) + 2) // 3, 0, 1, 6)
        layout.addWidget(telemetry_box)

        self.show_filtered = QtWidgets.QCheckBox(
            "Show time-aware low-pass speed (visualization only; raw safety path unchanged)")
        self.show_filtered.setChecked(True)
        layout.addWidget(self.show_filtered)

        plots = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.speed_plot = pg.PlotWidget(title="Speed vs time")
        self.speed_plot.setLabel("left", "Speed", units="RPM"); self.speed_plot.setLabel("bottom", "Time", units="s")
        self.actual_curve = self.speed_plot.plot(pen=pg.mkPen("c", width=2), name="Raw 0x9C")
        self.filtered_curve = self.speed_plot.plot(pen=pg.mkPen("y", width=2), name="Filtered display")
        self.angle_speed_curve = self.speed_plot.plot(
            pen=pg.mkPen("m", width=2), name="Angle-derived average")
        self.target_curve = self.speed_plot.plot(pen=pg.mkPen("g", width=2), name="Target")
        self.limit_high = pg.InfiniteLine(GUI_HARD_SPEED_RPM, pen=pg.mkPen("r", style=QtCore.Qt.DashLine))
        self.limit_low = pg.InfiniteLine(-GUI_HARD_SPEED_RPM, pen=pg.mkPen("r", style=QtCore.Qt.DashLine))
        self.speed_plot.addItem(self.limit_high); self.speed_plot.addItem(self.limit_low); self.speed_plot.addLegend()
        self.angle_plot = pg.PlotWidget(title="Accumulated angle vs time")
        self.angle_plot.setLabel("left", "Angle", units="deg"); self.angle_plot.setLabel("bottom", "Time", units="s")
        self.angle_curve = self.angle_plot.plot(pen=pg.mkPen("y", width=2))
        plots.addWidget(self.speed_plot); plots.addWidget(self.angle_plot)
        layout.addWidget(plots, stretch=2)

        log_box = QtWidgets.QGroupBox("Event log")
        log_layout = QtWidgets.QVBoxLayout(log_box)
        self.event_log = QtWidgets.QPlainTextEdit(); self.event_log.setReadOnly(True); self.event_log.setMaximumBlockCount(2000)
        export_row = QtWidgets.QHBoxLayout(); self.csv_label = QtWidgets.QLabel("No session CSV yet")
        self.csv_label.setMinimumWidth(0)
        self.csv_label.setSizePolicy(QtWidgets.QSizePolicy.Policy.Ignored,
                                     QtWidgets.QSizePolicy.Policy.Preferred)
        self.csv_label.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.export_button = QtWidgets.QPushButton("EXPORT CSV…"); self.export_button.setEnabled(False)
        export_row.addWidget(self.csv_label, 1); export_row.addWidget(self.export_button)
        log_layout.addWidget(self.event_log); log_layout.addLayout(export_row)
        layout.addWidget(log_box, stretch=1)

        self.connect_button.clicked.connect(self._connect)
        self.disconnect_button.clicked.connect(self._disconnect)
        self.start_button.clicked.connect(self._start)
        self.go_home_button.clicked.connect(self._go_home)
        self.stop_button.clicked.connect(lambda: self.worker.request_stop("operator_stop"))
        self.export_button.clicked.connect(self._export_csv)
        self.pid_read_button.clicked.connect(self._read_pid)
        self.show_filtered.toggled.connect(self._toggle_filtered)
        self.home_read_button.clicked.connect(self._read_home_angles)
        self.home_go_button.clicked.connect(self._go_home)
        self.home_set_button.clicked.connect(self._set_home)
        self.home_direction_sign.currentIndexChanged.connect(self._update_theta)

    def _start_worker(self) -> None:
        self.thread = QtCore.QThread(self)
        self.worker = MotorControlWorker(simulate=self.simulate)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.worker.telemetry.connect(self._queue_telemetry)
        # Preserve every fresh 0x9C current sample even if the display cache
        # later coalesces it with an angle/status row.
        self.worker.telemetry.connect(self._torque_feedback)
        if self.joint_estimator is not None:
            # Every fresh worker signal reaches this slot; the 50 ms GUI display
            # cache is intentionally not used for ROS joint measurements.
            self.worker.telemetry.connect(self._publish_joint_sample)
        self.worker.event.connect(self._event)
        self.worker.state_changed.connect(self._state)
        self.worker.connection_changed.connect(self._connection)
        self.worker.session_finished.connect(self._session_finished)
        self.worker.csv_ready.connect(self._csv_ready)
        self.worker.authorization_required.connect(self._authorize_motion)
        self.worker.pid_received.connect(self._pid_received)
        self.worker.pid_error.connect(self._pid_error)
        self.worker.home_result.connect(self._home_received)
        self.worker.home_error.connect(self._home_error)
        self.worker.home_authorization_required.connect(self._authorize_home_write)
        self.worker.finished.connect(self.thread.quit)
        self.ui_refresh_timer = QtCore.QTimer(self)
        self.ui_refresh_timer.setInterval(50)
        self.ui_refresh_timer.timeout.connect(self._flush_telemetry)
        self.ui_refresh_timer.start()
        self.disable_button.clicked.connect(self.worker.request_disable)
        self.thread.start()

    def _connect(self) -> None:
        if self.joint_estimator is not None and self.motor_id.value() != 1:
            QtWidgets.QMessageBox.warning(
                self, "JointState publishing requires motor ID 1",
                "The current HOME reference is calibrated for CAN ID 0x141.")
            return
        self._event("INFO", "Opening connection; this does not enable motion")
        self.worker.connect_motor(self.device.text().strip(),
                                  self.bitrate.currentData(), self.motor_id.value())

    def _disconnect(self) -> None:
        self.worker.disconnect_motor()

    def _start(self) -> None:
        rpm, duration = self.speed.value(), self.duration.value()
        try:
            validate_motion_request(rpm, duration)
            if self.auto_home.isChecked():
                validate_auto_home_request(
                    self.home_tolerance.value(), self.auto_home_speed.value())
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "Invalid motion request", str(exc))
            return
        self.running = True
        self.plot_epoch = None
        self.home_status.setText("Current mechanical HOME position requires a fresh check after motion")
        self.home_status.setStyleSheet("color:#b06000;font-weight:bold")
        self._refresh_controls()
        self.worker.start_motion(
            rpm, duration, self.auto_home.isChecked(),
            self.home_tolerance.value(), self.auto_home_speed.value(),
            self.position_hold.isChecked())

    def _go_home(self) -> None:
        try:
            validate_auto_home_request(self.home_tolerance.value(),
                                       self.auto_home_speed.value())
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "Invalid HOME request", str(exc))
            return
        self.running = True
        self.plot_epoch = None
        self.home_status.setText("Current mechanical HOME position requires a fresh check after motion")
        self.home_status.setStyleSheet("color:#b06000;font-weight:bold")
        self._refresh_controls()
        self.worker.request_home_motion(self.home_tolerance.value(),
                                        self.auto_home_speed.value(),
                                        self.position_hold.isChecked())

    @QtCore.Slot(str)
    def _authorize_motion(self, preflight_message: str) -> None:
        warning = (preflight_message +
                   "\n\nSoftware STOP cannot guarantee physical stopping.\n\n"
                   f"Type exactly:\n{GUI_CONFIRMATION}")
        phrase, accepted = QtWidgets.QInputDialog.getText(
            self, "Authorize bounded experimental motion", warning)
        response = phrase if accepted else ""
        if response != GUI_CONFIRMATION:
            self._event("WARNING", "Motion authorization declined")
        self.worker.submit_authorization(response)

    @QtCore.Slot(bool, str)
    def _connection(self, connected: bool, label: str) -> None:
        self.connected = connected
        self._last_torque_feedback_mono = None
        self._set_value("torque_current", None)
        self._set_value("torque_age", None)
        if self.joint_estimator is not None:
            self.joint_estimator.clear()
            if connected:
                self._joint_reference_pending_reconnect = False
                self.joint_estimator.direction_sign = self.home_direction_sign.currentData()
                # Read 0x94 and 0x92 through the same queued serial worker.
                self.worker.request_home_angle_read()
                self._event("INFO", "Reading 0x94 HOME reference before /joint_states publication")
            else:
                self._event("INFO", "/joint_states publication paused: motor disconnected")
        self.connection_label.setText("CONNECTED" if connected else label.upper())
        self.connection_label.setStyleSheet("font-weight:bold;color:" + ("#287a3d" if connected else "#a33"))
        self.can_id_label.setText(label if connected else "—")
        if not connected:
            self.pid_loaded = False
        self._refresh_controls()

    @QtCore.Slot(str)
    def _state(self, state: str) -> None:
        if state in ("IDLE", "STOPPED_ENABLED", "POSITION_HOLD", "DISCONNECTED", "STOP_UNCONFIRMED"):
            self.running = False
        self._set_value("motor_state", state)
        self._update_theta()
        self._refresh_controls()

    def _refresh_controls(self) -> None:
        self.connect_button.setEnabled(not self.connected)
        self.disconnect_button.setEnabled(self.connected and not self.running and not self.home_busy)
        self.start_button.setEnabled(self.connected and not self.running and not self.home_busy)
        self.go_home_button.setEnabled(self.connected and not self.running and not self.home_busy)
        self.stop_button.setEnabled(self.connected and self.running)
        self.disable_button.setEnabled(self.connected)
        self.device.setEnabled(not self.connected); self.bitrate.setEnabled(not self.connected)
        self.motor_id.setEnabled(not self.connected); self.speed.setEnabled(not self.running)
        self.duration.setEnabled(not self.running)
        self.auto_home.setEnabled(not self.running)
        self.position_hold.setEnabled(not self.running)
        self.auto_home_speed.setEnabled(not self.running)
        self.pid_read_button.setEnabled(self.connected and not self.running and not self.home_busy)
        home_enabled = self.connected and not self.running and not self.home_busy
        self.home_read_button.setEnabled(home_enabled)
        self.home_go_button.setEnabled(home_enabled)
        self.home_set_button.setEnabled(home_enabled)
        self.home_direction_sign.setEnabled(
            not self.home_busy and (self.joint_estimator is None or not self.connected))
        self.home_tolerance.setEnabled(not self.home_busy)
        for editor in self.pid_inputs.values():
            editor.setEnabled(self.pid_loaded)

    @QtCore.Slot(dict)
    def _pid_received(self, values: dict) -> None:
        for name in PID_FIELD_NAMES:
            self.pid_inputs[name].setValue(values[name])
        self.pid_loaded = True
        self.pid_status.setText(
            "Readback available. Values are raw uint8 under provisional V2.35 layout; "
            "MHF9025-V3 scaling remains unverified. No PID write command is available.")
        self._refresh_controls()

    @QtCore.Slot(str)
    def _pid_error(self, message: str) -> None:
        self.pid_loaded = False
        self.pid_status.setText("PID operation failed: " + message)
        self._refresh_controls()

    def _read_pid(self) -> None:
        self.pid_read_button.setEnabled(False)
        self.pid_status.setText("Reading PID parameters with 0x30…")
        self.worker.request_pid_read()

    def _begin_home_operation(self, message: str) -> None:
        self.home_busy = True
        self.home_status.setText(message)
        self._refresh_controls()

    def _read_home_angles(self) -> None:
        self._begin_home_operation("Reading fresh 0x94, 0x92, and 0x90 values…")
        self.worker.request_home_angle_read()

    def _set_home(self) -> None:
        self._begin_home_operation(
            "Preparing HOME write: confirming STOP/DISABLE and fresh stationary data…")
        self.worker.request_set_home()

    @QtCore.Slot(dict)
    def _authorize_home_write(self, summary: dict) -> None:
        warning = (
            "This operation permanently changes the motor encoder zero in ROM.\n\n"
            "Confirm the LiDAR is physically aligned with the desired mechanical HOME, "
            "the rotating assembly is safe, and no controller can issue motion.\n\n"
            f"Current absolute angle: {summary['single_turn_degrees']:.2f} deg\n"
            f"Current accumulated angle: {summary['multi_turn_degrees']:.2f} deg\n"
            f"Proposed CAN frame: ID=0x141 DLC=8 DATA={summary['frame_hex']}\n\n"
            "The command is sent once and is never retried automatically. A motor power cycle "
            f"is required before verification.\n\nType exactly:\n{FINAL_CONFIRMATION}"
        )
        phrase, accepted = QtWidgets.QInputDialog.getText(
            self, "Authorize persistent Absolute HOME write", warning)
        response = phrase if accepted else ""
        if response != FINAL_CONFIRMATION:
            self._event("WARNING", "Persistent HOME authorization declined")
        self.worker.submit_home_authorization(response)

    @QtCore.Slot(dict)
    def _home_received(self, result: dict) -> None:
        self.home_busy = False
        angles = result.get("angles") or {}
        if (self.joint_estimator is not None and
                result.get("state") in ("WRITE_ACKNOWLEDGED_UNVERIFIED", "WRITE_RESULT_UNKNOWN") and
                result.get("write_attempted_this_connection")):
            self._joint_reference_pending_reconnect = True
            self.joint_estimator.clear()
            self._event("WARNING", "/joint_states paused after HOME write; power-cycle and reconnect")
        if (self.joint_estimator is not None and
                not self._joint_reference_pending_reconnect and {
                "single_turn_degrees", "multi_turn_degrees"} <= angles.keys()):
            try:
                offset = self.joint_estimator.set_reference(
                    angles["single_turn_degrees"], angles["multi_turn_degrees"])
            except ValueError as exc:
                self._event("ERROR", "/joint_states HOME reference invalid: " + str(exc))
            else:
                self._event("INFO", f"/joint_states HOME reference ready; "
                            f"0x94-to-0x92 offset={offset:+.2f} deg")
        if "single_turn_degrees" in angles:
            self._home_single_degrees = angles["single_turn_degrees"]
            self.home_single.setText(f"{self._home_single_degrees:.2f} deg")
        if "multi_turn_degrees" in angles:
            self._home_multi_degrees = angles["multi_turn_degrees"]
            self.home_multi.setText(f"{self._home_multi_degrees:.2f} deg")
        verification = result.get("verification") or {}
        current_check = result.get("current_home_check") or {}
        if (result.get("state") == "VERIFIED_AFTER_POWER_CYCLE" and
                verification.get("passed") and current_check.get("passed") and
                current_check.get("mechanical_home_position_operator_attested")):
            status = "HOME VERIFIED AFTER POWER CYCLE AND PHYSICAL MARK CONFIRMATION"
            self.home_status.setStyleSheet("color:#287a3d;font-weight:bold")
        elif result.get("state") == "VERIFIED_AFTER_POWER_CYCLE":
            if current_check and not current_check.get("passed"):
                status = "CURRENT HOME CHECK FAILED — shaft mark or encoder angle is outside tolerance"
                self.home_status.setStyleSheet("color:#a33;font-weight:bold")
            else:
                status = "HOME write passed a previous power-cycle check"
                self.home_status.setStyleSheet("color:#b06000;font-weight:bold")
        else:
            status = result.get("state", "NOT_CALIBRATED")
            if status == "WRITE_ACKNOWLEDGED_UNVERIFIED":
                status = ("HOME write acknowledged; persistence not checked after power cycle. "
                          "GO HOME and START remain available.")
            elif status == "WRITE_RESULT_UNKNOWN":
                status = "HOME write result unknown; check the marked shaft position and angle"
            elif result.get("session_path"):
                status += f" — log: {result['session_path']}"
            self.home_status.setStyleSheet("color:#b06000;font-weight:bold")
        self.home_status.setText(status)
        self._update_theta()
        self._refresh_controls()

    @QtCore.Slot(str)
    def _home_error(self, message: str) -> None:
        self.home_busy = False
        self.home_status.setText("HOME operation failed: " + message)
        self.home_status.setStyleSheet("color:#a33;font-weight:bold")
        self._event("ERROR", "HOME operation failed: " + message)
        self._refresh_controls()

    def _update_theta(self, *_args) -> None:
        sign = self.home_direction_sign.currentData()
        use_multi = self.running and self._home_multi_degrees is not None
        motor_angle = self._home_multi_degrees if use_multi else self._home_single_degrees
        if motor_angle is None:
            self.home_theta_deg.setText("— deg")
            self.home_theta_rad.setText("— rad")
            return
        # A single-turn reading of 270 degrees represents -90 degrees from
        # HOME; use the signed equivalent for a consistent joint-angle display.
        if not use_multi:
            motor_angle = ((motor_angle + 180.0) % 360.0) - 180.0
        theta = float(sign) * motor_angle
        self.home_theta_deg.setText(f"{theta:.2f} deg")
        self.home_theta_rad.setText(f"{math.radians(theta):.6f} rad")
        self.home_theta_source.setText(
            "0x92 continuous" if use_multi else "0x94 single-turn absolute")

    def _toggle_filtered(self, checked: bool) -> None:
        self.filtered_curve.setVisible(checked)

    def _set_value(self, key: str, value, precision: int | None = None) -> None:
        label, unit = self.values[key]
        if value is None:
            text = "—"
        elif precision is not None and isinstance(value, (float, int)):
            text = f"{value:.{precision}f}"
        else:
            text = str(value)
        display = text + (f" {unit}" if unit and value is not None else "")
        label.setText(display)
        label.setToolTip(display)

    @QtCore.Slot(dict)
    def _queue_telemetry(self, data: dict) -> None:
        self._latest_ui_telemetry = data

    @QtCore.Slot(dict)
    def _torque_feedback(self, data: dict) -> None:
        if not self.connected or data.get("sample_source") != "0x9C":
            return
        raw = data.get("torque_current_raw")
        mono = data.get("monotonic_timestamp")
        if not isinstance(raw, int) or not isinstance(mono, (int, float)) or not math.isfinite(mono):
            return
        self._last_torque_feedback_mono = mono
        self._set_value("torque_current", raw)
        self._set_value("torque_age", max(0.0, time.monotonic() - mono), 2)

    @QtCore.Slot(dict)
    def _publish_joint_sample(self, data: dict) -> None:
        if not self.connected or self.joint_estimator is None:
            return
        measurement = self.joint_estimator.consume(data)
        if measurement is None:
            return
        try:
            self.ros_joint_publisher.publish(measurement)
        except BaseException as exc:
            self.joint_estimator.clear()
            self._event("ERROR", f"/joint_states publish failed: {type(exc).__name__}: {exc}")

    def _flush_telemetry(self) -> None:
        if self._last_torque_feedback_mono is not None:
            age = max(0.0, time.monotonic() - self._last_torque_feedback_mono)
            self._set_value("torque_age", age, 2)
            if age > 0.5:
                self._set_value("torque_current", None)
        if self._latest_ui_telemetry is None:
            if self._displayed_ui_telemetry is not None:
                age = max(0.0, (self._displayed_ui_telemetry.get("feedback_age_s") or 0.0) +
                          time.monotonic() -
                          self._displayed_ui_telemetry["monotonic_timestamp"])
                self._set_value("feedback_age", age, 3)
            return
        data = self._latest_ui_telemetry
        self._latest_ui_telemetry = None
        self._displayed_ui_telemetry = data
        self._set_value("target", data.get("requested_rpm"), 2)
        self._set_value("actual", data.get("actual_rpm_provisional"), 2)
        filtered = data.get("filtered_speed_rpm_visual")
        self._set_value("filtered", filtered if self.show_filtered.isChecked() else None, 2)
        angle_speed = data.get("angle_derived_rpm_provisional")
        self._set_value("angle_speed", angle_speed, 2)
        self._set_value("tracking_error", data.get("raw_tracking_error_rpm"), 2)
        angle = data.get("angle_degrees_provisional")
        self._set_value("angle_deg", angle, 2)
        self._set_value("angle_rad", None if angle is None else angle * 3.141592653589793 / 180, 4)
        if angle is not None:
            self._home_multi_degrees = angle
            self.home_multi.setText(f"{angle:.2f} deg")
            if self.running:
                self._update_theta()
        self._set_value("raw_speed", data.get("raw_speed_dps"))
        self._set_value("encoder", data.get("encoder_raw"))
        self._set_value("temperature", data.get("temperature_c"))
        self._set_value("voltage", data.get("voltage_v"), 2)
        self._set_value("motor_state", data.get("motor_state"))
        self._set_value("protection", data.get("protection_state"))
        self._set_value("last_response", data.get("utc_timestamp"))
        self._set_value("feedback_age", data.get("feedback_age_s"), 3)
        self._set_value("speed_hz", data.get("speed_sample_hz"), 1)
        self._set_value("angle_hz", data.get("angle_sample_hz"), 1)
        self._set_value("communication", data.get("communication_status"))
        stop_text = data.get("stop_confirmation_detail") or "not evaluated"
        if data.get("physical_stop_verified") is False and stop_text != "not evaluated":
            stop_text += "; physical stop not verified"
        self._set_value("stop_confirmation", stop_text)
        mono = data.get("monotonic_timestamp")
        source = data.get("sample_source", "")
        if mono is not None and source in ("0x9C", "0x92"):
            if self.plot_epoch is None:
                self.plot_epoch = mono
            x = mono - self.plot_epoch
            actual = data.get("actual_rpm_provisional")
            target = data.get("requested_rpm")
            self.target_times.append(x); self.plot_target.append(float("nan") if target is None else target)
            if source == "0x9C" and actual is not None:
                self.raw_times.append(x); self.plot_raw.append(actual)
                self.filtered_times.append(x); self.plot_filtered.append(
                    float("nan") if filtered is None else filtered)
            if source == "0x92" and angle is not None:
                self.angle_times.append(x); self.plot_angle.append(angle)
                if angle_speed is not None:
                    self.angle_speed_times.append(x); self.plot_angle_speed.append(angle_speed)
            self.actual_curve.setData(self.raw_times, self.plot_raw)
            self.filtered_curve.setData(self.filtered_times, self.plot_filtered)
            self.angle_speed_curve.setData(self.angle_speed_times, self.plot_angle_speed)
            self.target_curve.setData(self.target_times, self.plot_target)
            self.angle_curve.setData(self.angle_times, self.plot_angle)
            if x > 20:
                self.speed_plot.setXRange(x - 20, x); self.angle_plot.setXRange(x - 20, x)

    @QtCore.Slot(str, str)
    def _event(self, level: str, message: str) -> None:
        stamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
        self.event_log.appendPlainText(f"{stamp} [{level}] {message}")

    @QtCore.Slot(bool, str)
    def _session_finished(self, ok: bool, message: str) -> None:
        self.running = False
        self._event("INFO" if ok else "ERROR", message)
        self._refresh_controls()

    @QtCore.Slot(str)
    def _csv_ready(self, path: str) -> None:
        self.last_csv_path = path
        self.csv_label.setText(path)
        self.csv_label.setToolTip(path)
        self.export_button.setEnabled(True)

    def _export_csv(self) -> None:
        destination, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Export telemetry CSV", "motor_telemetry.csv", "CSV files (*.csv)")
        if destination:
            try:
                shutil.copy2(self.last_csv_path, destination)
            except OSError as exc:
                QtWidgets.QMessageBox.critical(self, "CSV export failed", str(exc))
            else:
                self._event("INFO", f"CSV exported to {destination}")

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self.setEnabled(False)
        self.worker.request_shutdown()
        if not self.thread.wait(3500):
            QtWidgets.QMessageBox.critical(
                self, "Shutdown cleanup unconfirmed",
                "The worker did not finish bounded STOP/DISABLE cleanup. Use the independent power cutoff.")
            event.ignore()
            self.setEnabled(True)
            return
        if self.ros_joint_publisher is not None:
            self.ros_joint_publisher.close()
        event.accept()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--simulate", action="store_true",
                        help="use deterministic feedback and never open a serial/CAN device")
    parser.add_argument("--publish-joint-states", action="store_true",
                        help="publish fresh 0x92 feedback as ROS 2 /joint_states from this GUI process")
    args = parser.parse_args(argv)
    require_qt_platform_runtime()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    try:
        window = MotorControlWindow(simulate=args.simulate,
                                    publish_joint_states=args.publish_joint_states)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

