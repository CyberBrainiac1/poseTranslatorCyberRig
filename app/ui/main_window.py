from __future__ import annotations

import csv
from collections import deque
from math import degrees, radians
from pathlib import Path
from typing import Dict
import codecs

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PySide6.QtCore import QTimer
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QScrollArea,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from app.config_io import load_config, save_config
from app.flypt_io import SerialInput, UDPInput, list_serial_ports, parse_pitch_roll_line
from app.geometry_checks import analyze_workspace, check_geometry_consistency
from app.math_core import estimate_pose_from_cable_lengths, neutral_lengths, solve_pose_to_cable
from app.models import CABLES, Limits, RigConfig, SolveInput, SolveResult, SweepSettings, Vec3
from app.plots import plot_heatmap, plot_side_view, plot_top_view
from app.serial_output import SerialOutput, format_csv, format_output
from app.ui.widgets import AnchorEditor, CableScalarEditor, FloatEdit, IntEdit, Vec3Editor


def default_config() -> RigConfig:
    return RigConfig(
        moving_anchors={
            "FL": Vec3(0.360, 0.260, 0.000),
            "BL": Vec3(-0.340, 0.260, 0.000),
            "FR": Vec3(0.360, -0.260, 0.000),
            "BR": Vec3(-0.340, -0.260, 0.000),
        },
        fixed_points={
            "FL": Vec3(0.520, 0.510, 0.560),
            "BL": Vec3(-0.520, 0.510, 0.560),
            "FR": Vec3(0.520, -0.510, 0.560),
            "BR": Vec3(-0.520, -0.510, 0.560),
        },
        rotation_center=Vec3(0.000, 0.000, -0.040),
        spool_radii={"FL": 0.026, "BL": 0.024, "FR": 0.026, "BR": 0.024},
        winding_signs={"FL": 1, "BL": 1, "FR": 1, "BR": 1},
        counts_per_output_rev=None,
        motor_encoder_cpr=2048,
        gearbox_ratio=40,
        cable_weights={"FL": 1.0, "BL": 1.0, "FR": 1.0, "BR": 1.0},
        limits=Limits(
            cable_min=0.15,
            cable_max=1.40,
            motor_min_rad=-2.6,
            motor_max_rad=2.6,
            rms_error_max=0.003,
            slack_pay_out_threshold=0.008,
        ),
    )


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Cable Rig Pose Translator")
        self.resize(1620, 980)

        self.cfg = default_config()
        self.last_solve: SolveResult | None = None
        self.live_source = None
        self.output = None
        self.live_sample_index = 0
        self.live_plot_paused = False
        self.live_history = {
            "time": deque(maxlen=600),
            "pitch": deque(maxlen=600),
            "roll": deque(maxlen=600),
            "q_left": deque(maxlen=600),
            "q_right": deque(maxlen=600),
            "counts_left": deque(maxlen=600),
            "counts_right": deque(maxlen=600),
            "rms": deque(maxlen=600),
        }
        self.live_history.update({f"ideal_{k}": deque(maxlen=600) for k in CABLES})
        self.live_history.update({f"pred_{k}": deque(maxlen=600) for k in CABLES})

        tabs = QTabWidget()
        self.setCentralWidget(tabs)

        self.geometry_tab = self._build_geometry_tab()
        self.solve_tab = self._build_solve_tab()
        self.analyze_tab = self._build_analyze_tab()
        self.live_tab = self._build_live_tab()

        tabs.addTab(self.geometry_tab, "Geometry")
        tabs.addTab(self.solve_tab, "Solve")
        tabs.addTab(self.analyze_tab, "Analyze")
        tabs.addTab(self.live_tab, "Live FlyPT")

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self._poll_live_input)
        self.poll_timer.start(50)
        self.statusBar().showMessage("Ready")

        self._apply_cfg_to_inputs()
        self._zero_current_pose()

    def _set_anchor_editor(self, editor: AnchorEditor, values: Dict[str, Vec3]) -> None:
        for k, vec in values.items():
            e = editor.editors[k]
            e.x_edit.setText(str(vec.x))
            e.y_edit.setText(str(vec.y))
            e.z_edit.setText(str(vec.z))

    def _set_scalar_editor(self, editor: CableScalarEditor, values: Dict[str, float | int]) -> None:
        for k, v in values.items():
            editor.edits[k].setText(str(v))

    def _build_geometry_tab(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        container = QWidget()
        container_layout = QVBoxLayout(container)

        self.moving_editor = AnchorEditor("Moving anchors", self.cfg.moving_anchors)
        self.fixed_editor = AnchorEditor("Fixed cable exits", self.cfg.fixed_points)
        self.center_editor = Vec3Editor(self.cfg.rotation_center)

        center_box = QGroupBox("Rotation center")
        cform = QFormLayout(center_box)
        cform.addRow("Center [m]", self.center_editor)

        motor_box = QGroupBox("Motor grouping")
        motor_layout = QVBoxLayout(motor_box)
        motor_layout.addWidget(QLabel("Left motor drives FL + BL"))
        motor_layout.addWidget(QLabel("Right motor drives FR + BR"))

        self.spool_radius_editor = CableScalarEditor("Spool radii [m]", self.cfg.spool_radii)
        self.winding_editor = CableScalarEditor("Winding signs (+1/-1)", self.cfg.winding_signs, use_integer_editor=True)
        self.weight_editor = CableScalarEditor("Cable solve weights", self.cfg.cable_weights)

        limits_box = QGroupBox("Global limits")
        lform = QFormLayout(limits_box)
        self.cable_min_edit = FloatEdit(self.cfg.limits.cable_min)
        self.cable_max_edit = FloatEdit(self.cfg.limits.cable_max)
        self.motor_min_edit = FloatEdit(self.cfg.limits.motor_min_rad)
        self.motor_max_edit = FloatEdit(self.cfg.limits.motor_max_rad)
        self.rms_max_edit = FloatEdit(self.cfg.limits.rms_error_max)
        self.slack_threshold_edit = FloatEdit(self.cfg.limits.slack_pay_out_threshold)
        lform.addRow("Cable min [m]", self.cable_min_edit)
        lform.addRow("Cable max [m]", self.cable_max_edit)
        lform.addRow("Motor min [rad]", self.motor_min_edit)
        lform.addRow("Motor max [rad]", self.motor_max_edit)
        lform.addRow("Max RMS error [m]", self.rms_max_edit)
        lform.addRow("Slack pay-out threshold [m]", self.slack_threshold_edit)

        encoder_box = QGroupBox("Encoder model")
        eform = QFormLayout(encoder_box)
        self.counts_per_out_edit = FloatEdit(0)
        self.cpr_edit = FloatEdit(self.cfg.motor_encoder_cpr or 0)
        self.gear_edit = FloatEdit(self.cfg.gearbox_ratio or 0)
        self.effective_counts_label = QLabel("")
        self.counts_per_out_edit.setToolTip("Direct counts per output revolution. Leave 0 to derive from motor encoder CPR and gearbox ratio.")
        self.cpr_edit.setToolTip("Motor encoder counts per motor revolution.")
        self.gear_edit.setToolTip("Gearbox ratio used to convert motor counts to output-shaft counts.")
        eform.addRow("Counts/output rev", self.counts_per_out_edit)
        eform.addRow("Motor encoder CPR", self.cpr_edit)
        eform.addRow("Gearbox ratio", self.gear_edit)
        eform.addRow("Effective counts/output rev", self.effective_counts_label)

        btn_box = QHBoxLayout()
        self.save_btn = QPushButton("Save config")
        self.load_btn = QPushButton("Load config")
        self.geom_check_btn = QPushButton("Analyze geometry")
        self.save_btn.clicked.connect(self._save_config)
        self.load_btn.clicked.connect(self._load_config)
        self.geom_check_btn.clicked.connect(self._analyze_geometry)
        btn_box.addWidget(self.save_btn)
        btn_box.addWidget(self.load_btn)
        btn_box.addWidget(self.geom_check_btn)

        self.geometry_warnings = QPlainTextEdit()
        self.geometry_warnings.setReadOnly(True)

        for widget in [
            self.moving_editor,
            self.fixed_editor,
            center_box,
            motor_box,
            self.spool_radius_editor,
            self.winding_editor,
            self.weight_editor,
            limits_box,
            encoder_box,
        ]:
            container_layout.addWidget(widget)
        container_layout.addLayout(btn_box)
        container_layout.addWidget(QLabel("Geometry warnings"))
        container_layout.addWidget(self.geometry_warnings)
        container_layout.addStretch(1)

        scroll.setWidget(container)
        layout.addWidget(scroll)
        return page

    def _build_solve_tab(self) -> QWidget:
        page = QWidget()
        split = QSplitter()
        root = QHBoxLayout(page)
        root.addWidget(split)

        left = QWidget()
        l = QVBoxLayout(left)
        form_box = QGroupBox("Input / pose")
        form = QFormLayout(form_box)
        self.solve_mode = QComboBox()
        self.solve_mode.addItems(["Pose to Cable", "Manual Cable Solver"])
        self.pitch_edit = FloatEdit(0.0)
        self.roll_edit = FloatEdit(0.0)
        self.current_len_editor = CableScalarEditor("Current measured cable lengths [m]", {k: 0.0 for k in CABLES})
        self.current_left_counts = FloatEdit(0.0)
        self.current_right_counts = FloatEdit(0.0)
        self.use_live_input = QCheckBox("Use live FlyPT input")

        form.addRow("Mode", self.solve_mode)
        form.addRow("Pitch [deg]", self.pitch_edit)
        form.addRow("Roll [deg]", self.roll_edit)
        form.addRow("Current left counts", self.current_left_counts)
        form.addRow("Current right counts", self.current_right_counts)
        form.addRow(self.use_live_input)

        l.addWidget(form_box)
        l.addWidget(self.current_len_editor)

        btns = QHBoxLayout()
        self.solve_btn = QPushButton("Solve")
        self.zero_btn = QPushButton("Zero current pose")
        self.solve_geom_btn = QPushButton("Analyze geometry")
        self.solve_sweep_btn = QPushButton("Sweep workspace")
        self.solve_save_btn = QPushButton("Save config")
        self.solve_load_btn = QPushButton("Load config")
        self.solve_btn.clicked.connect(self._solve)
        self.zero_btn.clicked.connect(self._zero_current_pose)
        self.solve_geom_btn.clicked.connect(self._analyze_geometry)
        self.solve_sweep_btn.clicked.connect(self._sweep)
        self.solve_save_btn.clicked.connect(self._save_config)
        self.solve_load_btn.clicked.connect(self._load_config)
        for button in [
            self.solve_btn,
            self.zero_btn,
            self.solve_geom_btn,
            self.solve_sweep_btn,
            self.solve_save_btn,
            self.solve_load_btn,
        ]:
            btns.addWidget(button)
        l.addLayout(btns)

        self.result_summary = QPlainTextEdit()
        self.result_summary.setReadOnly(True)
        self.result_table = QTableWidget(0, 0)
        l.addWidget(QLabel("Results summary"))
        l.addWidget(self.result_summary)
        l.addWidget(QLabel("Detailed results"))
        l.addWidget(self.result_table)

        split.addWidget(left)

        fig = Figure(figsize=(6, 5))
        self.canvas = FigureCanvas(fig)
        self.ax_top = fig.add_subplot(211)
        self.ax_side = fig.add_subplot(212)
        split.addWidget(self.canvas)

        split.setSizes([920, 640])
        return page

    def _build_analyze_tab(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        top = QHBoxLayout()
        self.pitch_min = FloatEdit(-15)
        self.pitch_max = FloatEdit(15)
        self.roll_min = FloatEdit(-15)
        self.roll_max = FloatEdit(15)
        self.pitch_steps = IntEdit(25)
        self.roll_steps = IntEdit(25)
        self.sweep_btn = QPushButton("Sweep workspace")
        self.sweep_btn.clicked.connect(self._sweep)

        for label, widget in [
            ("Pitch min", self.pitch_min),
            ("Pitch max", self.pitch_max),
            ("Roll min", self.roll_min),
            ("Roll max", self.roll_max),
            ("Pitch steps", self.pitch_steps),
            ("Roll steps", self.roll_steps),
        ]:
            top.addWidget(QLabel(label))
            top.addWidget(widget)
        top.addWidget(self.sweep_btn)
        layout.addLayout(top)

        self.analysis_summary = QPlainTextEdit()
        self.analysis_summary.setReadOnly(True)
        layout.addWidget(self.analysis_summary)

        self.analysis_table = QTableWidget(0, 14)
        self.analysis_table.setHorizontalHeaderLabels(
            ["Pitch", "Roll", "dFL", "dBL", "dFR", "dBR", "qL", "qR", "rFL", "rBL", "rFR", "rBR", "RMS", "Flags"]
        )
        layout.addWidget(self.analysis_table)

        hfig = Figure(figsize=(6, 4))
        self.heat_canvas = FigureCanvas(hfig)
        self.ax_heat = hfig.add_subplot(111)
        layout.addWidget(self.heat_canvas)
        return w

    def _build_live_tab(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        io_box = QGroupBox("Live FlyPT companion")
        grid = QGridLayout(io_box)

        self.input_mode = QComboBox()
        self.input_mode.addItems(["manual", "serial", "udp"])
        self.serial_port = QComboBox()
        self.serial_port.addItems(list_serial_ports() or ["(none)"])
        self.serial_baud = IntEdit(115200)
        self.udp_host = QLineEdit("0.0.0.0")
        self.udp_port = IntEdit(9000)
        self.refresh_ports_btn = QPushButton("Refresh ports")
        self.refresh_ports_btn.clicked.connect(self._refresh_ports)

        self.output_mode = QComboBox()
        self.output_mode.addItems(["disabled", "serial", "csv"])
        self.out_template = QLineEdit(r"L={left_counts},R={right_counts}\n")
        self.connection_status = QLabel("Disconnected")
        self.latest_pose_label = QLabel("Latest pose: --")

        grid.addWidget(QLabel("Input mode"), 0, 0)
        grid.addWidget(self.input_mode, 0, 1)
        grid.addWidget(QLabel("Serial port"), 1, 0)
        grid.addWidget(self.serial_port, 1, 1)
        grid.addWidget(self.refresh_ports_btn, 1, 2)
        grid.addWidget(QLabel("Baud"), 2, 0)
        grid.addWidget(self.serial_baud, 2, 1)
        grid.addWidget(QLabel("UDP host"), 3, 0)
        grid.addWidget(self.udp_host, 3, 1)
        grid.addWidget(QLabel("UDP port"), 4, 0)
        grid.addWidget(self.udp_port, 4, 1)
        grid.addWidget(QLabel("Output mode"), 5, 0)
        grid.addWidget(self.output_mode, 5, 1)
        grid.addWidget(QLabel("Output template"), 6, 0)
        grid.addWidget(self.out_template, 6, 1, 1, 2)
        grid.addWidget(QLabel("Connection"), 7, 0)
        grid.addWidget(self.connection_status, 7, 1)
        grid.addWidget(QLabel("Latest received"), 8, 0)
        grid.addWidget(self.latest_pose_label, 8, 1, 1, 2)

        self.apply_live_btn = QPushButton("Apply Live I/O")
        self.apply_live_btn.clicked.connect(self._apply_live_io)
        grid.addWidget(self.apply_live_btn, 9, 0, 1, 2)

        layout.addWidget(io_box)

        controls = QHBoxLayout()
        self.pause_live_plot_btn = QPushButton("Pause graphs")
        self.clear_live_plot_btn = QPushButton("Clear graphs")
        self.save_live_csv_btn = QPushButton("Save live CSV")
        self.pause_live_plot_btn.clicked.connect(self._toggle_live_plot_pause)
        self.clear_live_plot_btn.clicked.connect(self._clear_live_history)
        self.save_live_csv_btn.clicked.connect(self._save_live_history_csv)
        controls.addWidget(self.pause_live_plot_btn)
        controls.addWidget(self.clear_live_plot_btn)
        controls.addWidget(self.save_live_csv_btn)
        layout.addLayout(controls)

        help_label = QLabel(
            "Accepted input lines: CSV `pitch_deg,roll_deg` or tagged `P=1.5,R=-2.2`.\n"
            "Live graphs show incoming pose, solved motor targets, ideal vs predicted cable deltas, and RMS error."
        )
        help_label.setWordWrap(True)
        layout.addWidget(help_label)

        live_fig = Figure(figsize=(8, 8))
        self.live_canvas = FigureCanvas(live_fig)
        self.ax_live_pose = live_fig.add_subplot(411)
        self.ax_live_motor = live_fig.add_subplot(412)
        self.ax_live_cable = live_fig.add_subplot(413)
        self.ax_live_rms = live_fig.add_subplot(414)
        live_fig.tight_layout()
        layout.addWidget(self.live_canvas)

        self.live_log = QPlainTextEdit()
        self.live_log.setReadOnly(True)
        layout.addWidget(self.live_log)

        return w

    def _read_cfg_from_inputs(self) -> RigConfig:
        cfg = RigConfig(
            moving_anchors=self.moving_editor.values(),
            fixed_points=self.fixed_editor.values(),
            rotation_center=self.center_editor.value(),
            spool_radii=self.spool_radius_editor.float_values(),
            winding_signs=self.winding_editor.int_values(),
            counts_per_output_rev=self.counts_per_out_edit.value() if self.counts_per_out_edit.value() > 0 else None,
            motor_encoder_cpr=self.cpr_edit.value() if self.cpr_edit.value() > 0 else None,
            gearbox_ratio=self.gear_edit.value() if self.gear_edit.value() > 0 else None,
            cable_weights=self.weight_editor.float_values(),
            limits=Limits(
                cable_min=self.cable_min_edit.value(),
                cable_max=self.cable_max_edit.value(),
                motor_min_rad=self.motor_min_edit.value(),
                motor_max_rad=self.motor_max_edit.value(),
                rms_error_max=self.rms_max_edit.value(),
                slack_pay_out_threshold=self.slack_threshold_edit.value(),
            ),
        )
        cfg.validate()
        return cfg

    def _apply_cfg_to_inputs(self) -> None:
        self.moving_editor.set_values(self.cfg.moving_anchors)
        self.fixed_editor.set_values(self.cfg.fixed_points)
        self.center_editor.set_value(self.cfg.rotation_center)
        self.spool_radius_editor.set_values(self.cfg.spool_radii)
        self.winding_editor.set_values(self.cfg.winding_signs)
        self.weight_editor.set_values(self.cfg.cable_weights)
        self.counts_per_out_edit.set_value(self.cfg.counts_per_output_rev or 0)
        self.cpr_edit.set_value(self.cfg.motor_encoder_cpr or 0)
        self.gear_edit.set_value(self.cfg.gearbox_ratio or 0)
        self.cable_min_edit.set_value(self.cfg.limits.cable_min)
        self.cable_max_edit.set_value(self.cfg.limits.cable_max)
        self.motor_min_edit.set_value(self.cfg.limits.motor_min_rad)
        self.motor_max_edit.set_value(self.cfg.limits.motor_max_rad)
        self.rms_max_edit.set_value(self.cfg.limits.rms_error_max)
        self.slack_threshold_edit.set_value(self.cfg.limits.slack_pay_out_threshold)
        self._update_effective_counts_label()

    def _update_effective_counts_label(self) -> None:
        try:
            counts = self._read_cfg_from_inputs().effective_counts_per_output_rev()
        except Exception:
            counts = None
        self.effective_counts_label.setText(f"{counts:.2f}" if counts else "Not configured")

    def _save_config(self) -> None:
        try:
            cfg = self._read_cfg_from_inputs()
            path, _ = QFileDialog.getSaveFileName(self, "Save config", "", "JSON (*.json)")
            if path:
                save_config(path, cfg)
                self.statusBar().showMessage(f"Saved {path}")
                self.cfg = cfg
                self._update_effective_counts_label()
        except Exception as exc:
            QMessageBox.critical(self, "Save config failed", str(exc))

    def _load_config(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Load config", "", "JSON (*.json)")
        if not path:
            return
        try:
            cfg = load_config(path)
            self.cfg = cfg
            self._apply_cfg_to_inputs()
            self.statusBar().showMessage(f"Loaded {path}")
            self._zero_current_pose()
        except Exception as exc:
            QMessageBox.critical(self, "Load config failed", str(exc))

    def _analyze_geometry(self) -> None:
        try:
            self.cfg = self._read_cfg_from_inputs()
            out = check_geometry_consistency(self.cfg)
            lines = [
                f"Asymmetry score: {out.asymmetric_score:.6f}",
                "Neutral cable lengths [m]: " + ", ".join(f"{k}={out.neutral_lengths[k]:.4f}" for k in CABLES),
            ]
            if out.warnings:
                lines.append("Warnings:")
                lines.extend(f"- {warning}" for warning in out.warnings)
            else:
                lines.append("No geometry warnings detected.")
            self.geometry_warnings.setPlainText("\n".join(lines))
            self.statusBar().showMessage("Geometry analysis complete")
        except Exception as exc:
            QMessageBox.critical(self, "Geometry analysis failed", str(exc))

    def _zero_current_pose(self) -> None:
        try:
            self.cfg = self._read_cfg_from_inputs()
            l0 = neutral_lengths(self.cfg)
            self.current_len_editor.set_values({k: f"{l0[k]:.6f}" for k in CABLES})
            self.pitch_edit.set_value(0.0)
            self.roll_edit.set_value(0.0)
            self.current_left_counts.set_value(0.0)
            self.current_right_counts.set_value(0.0)
            self._render_pose_plots(0.0, 0.0, {k: True for k in CABLES})
            self.statusBar().showMessage("Current pose zeroed to neutral")
        except Exception as exc:
            QMessageBox.critical(self, "Zero failed", str(exc))

    def _solve(self) -> None:
        try:
            self.cfg = self._read_cfg_from_inputs()
            lengths = self.current_len_editor.float_values()
            if self.solve_mode.currentText() == "Manual Cable Solver":
                pose = estimate_pose_from_cable_lengths(self.cfg, lengths)
                self.pitch_edit.set_value(pose.pitch_deg)
                self.roll_edit.set_value(pose.roll_deg)
                self._show_manual_result(lengths, pose)
                self.statusBar().showMessage("Manual cable solve complete")
                return

            solve_input = SolveInput(
                pitch_deg=self.pitch_edit.value(),
                roll_deg=self.roll_edit.value(),
                current_cable_lengths=lengths,
                current_counts_left=self.current_left_counts.value(),
                current_counts_right=self.current_right_counts.value(),
            )
            res = solve_pose_to_cable(self.cfg, solve_input)
            self.last_solve = res
            self._show_solve_result(res)
            self._send_output_if_enabled(res)
            self._append_live_history(self.pitch_edit.value(), self.roll_edit.value(), res)
            self.statusBar().showMessage("Pose solve complete")
        except Exception as exc:
            QMessageBox.critical(self, "Solve failed", str(exc))

    def _show_solve_result(self, res) -> None:
        headers = ["Cable", "Neutral [m]", "Target [m]", "Ideal dL [m]", "Pred dL [m]", "Residual [m]", "Req q [rad]"]
        self.result_table.setColumnCount(len(headers))
        self.result_table.setHorizontalHeaderLabels(headers)
        self.result_table.setRowCount(len(CABLES))
        for row, key in enumerate(CABLES):
            values = [
                key,
                f"{res.neutral_lengths[key]:.6f}",
                f"{res.target_lengths[key]:.6f}",
                f"{res.ideal_deltas[key]:+.6f}",
                f"{res.predicted_deltas[key]:+.6f}",
                f"{res.residuals[key]:+.6f}",
                f"{res.per_cable_requested_angles_rad[key]:+.5f}",
            ]
            for col, value in enumerate(values):
                self.result_table.setItem(row, col, QTableWidgetItem(value))

        lines = [
            f"qL={res.q_left_rad:.6f} rad ({degrees(res.q_left_rad):.3f} deg)",
            f"qR={res.q_right_rad:.6f} rad ({degrees(res.q_right_rad):.3f} deg)",
            f"Left side mismatch: {res.side_mismatch_rad['left']:.6f} rad",
            f"Right side mismatch: {res.side_mismatch_rad['right']:.6f} rad",
            f"RMS error [m]: {res.rms_error:.6f}",
            f"Counts/output rev: {res.counts_per_output_rev if res.counts_per_output_rev is not None else 'not configured'}",
            f"Target counts: L={res.target_counts_left}, R={res.target_counts_right}",
            f"Delta counts: L={res.delta_counts_left}, R={res.delta_counts_right}",
            f"Clamped counts: L={res.clamped_counts_left}, R={res.clamped_counts_right}",
        ]
        if res.warnings:
            lines.append("Warnings:")
            lines.extend(f"- {warning}" for warning in res.warnings)
        self.result_summary.setPlainText("\n".join(lines))

        ok = {k: self.cfg.limits.cable_min <= res.target_lengths[k] <= self.cfg.limits.cable_max for k in CABLES}
        self._render_pose_plots(self.roll_edit.value(), self.pitch_edit.value(), ok)

    def _show_manual_result(self, lengths, pose) -> None:
        headers = ["Cable", "Measured [m]", "Predicted [m]", "Residual [m]"]
        self.result_table.setColumnCount(len(headers))
        self.result_table.setHorizontalHeaderLabels(headers)
        self.result_table.setRowCount(len(CABLES))
        for row, key in enumerate(CABLES):
            values = [
                key,
                f"{lengths[key]:.6f}",
                f"{pose.predicted_lengths[key]:.6f}",
                f"{pose.residuals[key]:+.6f}",
            ]
            for col, value in enumerate(values):
                self.result_table.setItem(row, col, QTableWidgetItem(value))
        lines = [
            f"Estimated pitch: {pose.pitch_deg:.4f} deg",
            f"Estimated roll: {pose.roll_deg:.4f} deg",
            f"Residual RMS [m]: {pose.residual_rms:.6f}",
            f"Valid: {'yes' if pose.valid else 'no'}",
        ]
        if pose.warnings:
            lines.append("Warnings:")
            lines.extend(f"- {warning}" for warning in pose.warnings)
        self.result_summary.setPlainText("\n".join(lines))
        ok = {k: self.cfg.limits.cable_min <= pose.predicted_lengths[k] <= self.cfg.limits.cable_max for k in CABLES}
        self._render_pose_plots(pose.roll_deg, pose.pitch_deg, ok)

    def _render_pose_plots(self, roll_deg: float, pitch_deg: float, ok: Dict[str, bool]) -> None:
        plot_top_view(self.ax_top, self.cfg, radians(roll_deg), radians(pitch_deg), ok)
        plot_side_view(self.ax_side, self.cfg, radians(roll_deg), radians(pitch_deg), ok)
        self.canvas.draw_idle()

    def _sweep(self) -> None:
        try:
            self.cfg = self._read_cfg_from_inputs()
            settings = SweepSettings(
                pitch_min_deg=self.pitch_min.value(),
                pitch_max_deg=self.pitch_max.value(),
                roll_min_deg=self.roll_min.value(),
                roll_max_deg=self.roll_max.value(),
                pitch_steps=max(2, self.pitch_steps.value()),
                roll_steps=max(2, self.roll_steps.value()),
            )
            result = analyze_workspace(self.cfg, settings)
            lines = [
                f"Max RMS: {result.max_rms:.6f} m",
                f"Mean RMS: {result.mean_rms:.6f} m",
                f"Worst pose (pitch, roll): ({result.worst_pose[0]:.2f}, {result.worst_pose[1]:.2f}) deg",
                f"Valid poses: {result.valid_pct:.2f} %",
                "Cable ranges [m]: " + ", ".join(
                    f"{key}=({result.cable_length_ranges[key][0]:.4f}, {result.cable_length_ranges[key][1]:.4f})" for key in CABLES
                ),
                "Motor ranges [rad]: " + ", ".join(
                    f"{side}=({bounds[0]:.4f}, {bounds[1]:.4f})" for side, bounds in result.motor_angle_range.items()
                ),
            ]
            self.analysis_summary.setPlainText("\n".join(lines))
            self.analysis_table.setRowCount(min(600, len(result.rows)))
            for i, row in enumerate(result.rows[:600]):
                for col, val in enumerate([
                    f"{row.pitch_deg:.2f}",
                    f"{row.roll_deg:.2f}",
                    f"{row.ideal_deltas['FL']:+.5f}",
                    f"{row.ideal_deltas['BL']:+.5f}",
                    f"{row.ideal_deltas['FR']:+.5f}",
                    f"{row.ideal_deltas['BR']:+.5f}",
                    f"{row.q_left_rad:+.5f}",
                    f"{row.q_right_rad:+.5f}",
                    f"{row.residuals['FL']:+.5f}",
                    f"{row.residuals['BL']:+.5f}",
                    f"{row.residuals['FR']:+.5f}",
                    f"{row.residuals['BR']:+.5f}",
                    f"{row.rms_error:.6f}",
                    "; ".join(row.flags) if row.flags else "OK",
                ]):
                    self.analysis_table.setItem(i, col, QTableWidgetItem(val))

            plot_heatmap(
                self.ax_heat,
                result.heatmap,
                settings.pitch_min_deg,
                settings.pitch_max_deg,
                settings.roll_min_deg,
                settings.roll_max_deg,
            )
            self.heat_canvas.draw_idle()
            self.statusBar().showMessage("Workspace sweep complete")
        except Exception as exc:
            QMessageBox.critical(self, "Sweep failed", str(exc))

    def _refresh_ports(self) -> None:
        self.serial_port.clear()
        self.serial_port.addItems(list_serial_ports() or ["(none)"])
        self.statusBar().showMessage("Serial ports refreshed")

    def _shutdown_live_io(self) -> None:
        if self.live_source is not None:
            self.live_source.close()
            self.live_source = None
        if self.output is not None:
            self.output.close()
            self.output = None
        self.connection_status.setText("Disconnected")

    def _apply_live_io(self) -> None:
        self._shutdown_live_io()

        try:
            mode = self.input_mode.currentText()
            if mode == "serial":
                p = self.serial_port.currentText()
                if p and p != "(none)":
                    src = SerialInput(port=p, baud=self.serial_baud.value())
                    src.open()
                    self.live_source = src
                    self.connection_status.setText(f"Serial {p} @ {self.serial_baud.value()}")
            elif mode == "udp":
                src = UDPInput(host=self.udp_host.text().strip() or "0.0.0.0", port=self.udp_port.value())
                src.open()
                self.live_source = src
                self.connection_status.setText(f"UDP {self.udp_host.text().strip() or '0.0.0.0'}:{self.udp_port.value()}")
            else:
                self.connection_status.setText("Manual mode")

            out_mode = self.output_mode.currentText()
            if out_mode == "serial":
                p = self.serial_port.currentText()
                if p and p != "(none)":
                    self.output = SerialOutput(port=p, baud=self.serial_baud.value())
                    self.output.open()

            self.statusBar().showMessage("Live I/O applied")
        except Exception as exc:
            self._shutdown_live_io()
            QMessageBox.critical(self, "Live I/O error", str(exc))

    def _append_live_history(self, pitch_deg: float, roll_deg: float, res: SolveResult) -> None:
        if self.live_plot_paused:
            return
        self.live_sample_index += 1
        self.live_history["time"].append(self.live_sample_index)
        self.live_history["pitch"].append(pitch_deg)
        self.live_history["roll"].append(roll_deg)
        self.live_history["q_left"].append(res.q_left_rad)
        self.live_history["q_right"].append(res.q_right_rad)
        self.live_history["counts_left"].append(res.clamped_counts_left if res.clamped_counts_left is not None else res.target_counts_left or 0.0)
        self.live_history["counts_right"].append(res.clamped_counts_right if res.clamped_counts_right is not None else res.target_counts_right or 0.0)
        self.live_history["rms"].append(res.rms_error)
        for key in CABLES:
            self.live_history[f"ideal_{key}"].append(res.ideal_deltas[key])
            self.live_history[f"pred_{key}"].append(res.predicted_deltas[key])
        self._refresh_live_plots()

    def _refresh_live_plots(self) -> None:
        time_values = list(self.live_history["time"])
        for axis in [self.ax_live_pose, self.ax_live_motor, self.ax_live_cable, self.ax_live_rms]:
            axis.clear()
        if not time_values:
            self.live_canvas.draw_idle()
            return

        self.ax_live_pose.plot(time_values, list(self.live_history["pitch"]), label="Pitch [deg]")
        self.ax_live_pose.plot(time_values, list(self.live_history["roll"]), label="Roll [deg]")
        self.ax_live_pose.legend(loc="upper left")
        self.ax_live_pose.set_title("Incoming FlyPT pose")
        self.ax_live_pose.grid(True, alpha=0.3)

        self.ax_live_motor.plot(time_values, list(self.live_history["q_left"]), label="qL [rad]")
        self.ax_live_motor.plot(time_values, list(self.live_history["q_right"]), label="qR [rad]")
        self.ax_live_motor.plot(time_values, list(self.live_history["counts_left"]), label="Left counts")
        self.ax_live_motor.plot(time_values, list(self.live_history["counts_right"]), label="Right counts")
        self.ax_live_motor.legend(loc="upper left", ncol=2)
        self.ax_live_motor.set_title("Solved motor targets")
        self.ax_live_motor.grid(True, alpha=0.3)

        for key in CABLES:
            self.ax_live_cable.plot(time_values, list(self.live_history[f"ideal_{key}"]), label=f"{key} ideal")
            self.ax_live_cable.plot(time_values, list(self.live_history[f"pred_{key}"]), linestyle="--", label=f"{key} pred")
        self.ax_live_cable.legend(loc="upper left", ncol=4, fontsize=8)
        self.ax_live_cable.set_title("Ideal vs predicted cable deltas")
        self.ax_live_cable.grid(True, alpha=0.3)

        self.ax_live_rms.plot(time_values, list(self.live_history["rms"]), label="RMS error [m]", color="tab:red")
        self.ax_live_rms.legend(loc="upper left")
        self.ax_live_rms.set_title("Coupling RMS error")
        self.ax_live_rms.set_xlabel("Sample")
        self.ax_live_rms.grid(True, alpha=0.3)
        self.live_canvas.draw_idle()

    def _toggle_live_plot_pause(self) -> None:
        self.live_plot_paused = not self.live_plot_paused
        self.pause_live_plot_btn.setText("Resume graphs" if self.live_plot_paused else "Pause graphs")

    def _clear_live_history(self) -> None:
        for series in self.live_history.values():
            series.clear()
        self.live_sample_index = 0
        self._refresh_live_plots()
        self.statusBar().showMessage("Live graph history cleared")

    def _save_live_history_csv(self) -> None:
        if not self.live_history["time"]:
            QMessageBox.information(self, "No live data", "No live graph data has been captured yet.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save live history", str(Path.cwd() / "live_history.csv"), "CSV (*.csv)")
        if not path:
            return
        headers = [
            "sample",
            "pitch_deg",
            "roll_deg",
            "q_left_rad",
            "q_right_rad",
            "counts_left",
            "counts_right",
            "rms_error",
        ] + [f"ideal_{key}" for key in CABLES] + [f"pred_{key}" for key in CABLES]
        history_lists = {key: list(values) for key, values in self.live_history.items()}
        with Path(path).open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(headers)
            for idx in range(len(history_lists["time"])):
                row = [
                    history_lists["time"][idx],
                    history_lists["pitch"][idx],
                    history_lists["roll"][idx],
                    history_lists["q_left"][idx],
                    history_lists["q_right"][idx],
                    history_lists["counts_left"][idx],
                    history_lists["counts_right"][idx],
                    history_lists["rms"][idx],
                ]
                row.extend(history_lists[f"ideal_{key}"][idx] for key in CABLES)
                row.extend(history_lists[f"pred_{key}"][idx] for key in CABLES)
                writer.writerow(row)
        self.statusBar().showMessage(f"Saved live history to {path}")

    def _poll_live_input(self) -> None:
        if not self.use_live_input.isChecked() or self.live_source is None:
            return
        try:
            line = self.live_source.poll_line()
            if not line:
                return
            parsed = parse_pitch_roll_line(line)
            if not parsed:
                self.live_log.appendPlainText(f"Unparsed input: {line}")
                return
            pitch, roll = parsed
            self.latest_pose_label.setText(f"Latest pose: pitch={pitch:.3f} deg, roll={roll:.3f} deg")
            self.pitch_edit.set_value(pitch)
            self.roll_edit.set_value(roll)
            self._solve()
            self.live_log.appendPlainText(line)
        except Exception as exc:
            self.live_log.appendPlainText(f"Live input error: {exc}")

    def _send_output_if_enabled(self, res) -> None:
        left_counts = res.clamped_counts_left if res.clamped_counts_left is not None else res.target_counts_left
        right_counts = res.clamped_counts_right if res.clamped_counts_right is not None else res.target_counts_right
        if left_counts is None or right_counts is None:
            return
        mode = self.output_mode.currentText()
        if mode == "disabled":
            return
        if mode == "csv":
            payload = format_csv(left_counts, right_counts)
            self.live_log.appendPlainText(payload.strip())
            return
        if mode == "serial" and self.output is not None:
            template_text = self.out_template.text()
            try:
                template = codecs.decode(template_text, "unicode_escape")
            except UnicodeDecodeError:
                template = template_text
                self.statusBar().showMessage("Output template escape decoding failed; using raw template text")
            payload = format_output(template, left_counts, right_counts, res.delta_counts_left, res.delta_counts_right)
            self.output.send_line(payload)
            self.live_log.appendPlainText(payload.strip())

    def closeEvent(self, event: QCloseEvent) -> None:
        self._shutdown_live_io()
        super().closeEvent(event)
