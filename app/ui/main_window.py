from __future__ import annotations

from math import radians
from pathlib import Path
from typing import Dict, Optional
import codecs

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PySide6.QtCore import QTimer
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
    QSpinBox,
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
from app.models import CABLES, RigConfig, SolveInput, SweepSettings, Vec3
from app.plots import plot_heatmap, plot_side_view, plot_top_view
from app.serial_output import SerialOutput, format_csv, format_output
from app.ui.widgets import AnchorEditor, CableScalarEditor, FloatEdit, IntEdit, Vec3Editor


def default_config() -> RigConfig:
    return RigConfig(
        moving_anchors={
            "FL": Vec3(0.35, 0.25, 0.0),
            "BL": Vec3(-0.35, 0.25, 0.0),
            "FR": Vec3(0.35, -0.25, 0.0),
            "BR": Vec3(-0.35, -0.25, 0.0),
        },
        fixed_points={
            "FL": Vec3(0.5, 0.5, 0.5),
            "BL": Vec3(-0.5, 0.5, 0.5),
            "FR": Vec3(0.5, -0.5, 0.5),
            "BR": Vec3(-0.5, -0.5, 0.5),
        },
        rotation_center=Vec3(0.0, 0.0, 0.0),
        spool_radii={"FL": 0.025, "BL": 0.025, "FR": 0.025, "BR": 0.025},
        winding_signs={"FL": 1, "BL": 1, "FR": 1, "BR": 1},
        counts_per_output_rev=4096,
    )


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Cable Rig Pose Translator")
        self.resize(1400, 900)

        self.cfg = default_config()
        self.last_solve = None
        self.live_source = None
        self.output = None

        tabs = QTabWidget()
        self.setCentralWidget(tabs)

        self.geometry_tab = self._build_geometry_tab()
        self.solve_tab = self._build_solve_tab()
        self.analyze_tab = self._build_analyze_tab()
        self.live_tab = self._build_live_tab()

        tabs.addTab(self.geometry_tab, "Geometry")
        tabs.addTab(self.solve_tab, "Solve")
        tabs.addTab(self.analyze_tab, "Analyze")
        tabs.addTab(self.live_tab, "Live I/O")

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
        w = QWidget()
        layout = QHBoxLayout(w)

        left = QWidget()
        left_l = QVBoxLayout(left)
        self.moving_editor = AnchorEditor("Moving anchors", default_config().moving_anchors)
        self.fixed_editor = AnchorEditor("Fixed cable exits", default_config().fixed_points)
        self.center_editor = Vec3Editor(default_config().rotation_center)

        center_box = QGroupBox("Rotation center")
        cform = QFormLayout(center_box)
        cform.addRow("Center [m]", self.center_editor)

        self.spool_radius_editor = CableScalarEditor("Spool radii [m]", default_config().spool_radii)
        self.winding_editor = CableScalarEditor("Winding signs (+1/-1)", default_config().winding_signs, is_int=True)

        limits_box = QGroupBox("Global limits")
        lform = QFormLayout(limits_box)
        self.cable_min_edit = FloatEdit(0.05)
        self.cable_max_edit = FloatEdit(2.0)
        self.motor_min_edit = FloatEdit(-3.14)
        self.motor_max_edit = FloatEdit(3.14)
        self.rms_max_edit = FloatEdit(0.005)
        lform.addRow("Cable min [m]", self.cable_min_edit)
        lform.addRow("Cable max [m]", self.cable_max_edit)
        lform.addRow("Motor min [rad]", self.motor_min_edit)
        lform.addRow("Motor max [rad]", self.motor_max_edit)
        lform.addRow("Max RMS error [m]", self.rms_max_edit)

        encoder_box = QGroupBox("Encoder model")
        eform = QFormLayout(encoder_box)
        self.counts_per_out_edit = FloatEdit(4096)
        self.cpr_edit = FloatEdit(0)
        self.gear_edit = FloatEdit(0)
        self.counts_per_out_edit.setToolTip("Encoder counts per output shaft revolution")
        self.cpr_edit.setToolTip("Motor encoder CPR, used with gearbox ratio if counts/output rev is not set")
        self.gear_edit.setToolTip("Output revolutions per motor revolution multiplier")
        eform.addRow("Counts/output rev", self.counts_per_out_edit)
        eform.addRow("Motor encoder CPR (optional)", self.cpr_edit)
        eform.addRow("Gearbox ratio (optional)", self.gear_edit)

        left_l.addWidget(self.moving_editor)
        left_l.addWidget(self.fixed_editor)
        left_l.addWidget(center_box)
        left_l.addWidget(self.spool_radius_editor)
        left_l.addWidget(self.winding_editor)
        left_l.addWidget(limits_box)
        left_l.addWidget(encoder_box)

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
        left_l.addLayout(btn_box)

        self.geometry_warnings = QPlainTextEdit()
        self.geometry_warnings.setReadOnly(True)
        left_l.addWidget(QLabel("Geometry warnings"))
        left_l.addWidget(self.geometry_warnings)

        layout.addWidget(left)
        return w

    def _build_solve_tab(self) -> QWidget:
        w = QWidget()
        split = QSplitter()
        root = QHBoxLayout(w)
        root.addWidget(split)

        left = QWidget()
        l = QVBoxLayout(left)
        form_box = QGroupBox("Input / pose")
        form = QFormLayout(form_box)
        self.solve_mode = QComboBox()
        self.solve_mode.addItems(["Pose to Cable", "Manual Cable Solver"])
        self.pitch_edit = FloatEdit(0)
        self.roll_edit = FloatEdit(0)
        self.pitch_edit.setToolTip("Desired platform pitch angle in degrees")
        self.roll_edit.setToolTip("Desired platform roll angle in degrees")
        self.current_len_editor = CableScalarEditor("Current measured cable lengths [m]", {k: 1.0 for k in CABLES})
        self.current_left_counts = FloatEdit(0)
        self.current_right_counts = FloatEdit(0)
        self.current_left_counts.setToolTip("Current measured encoder count for left motor")
        self.current_right_counts.setToolTip("Current measured encoder count for right motor")
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
        self.solve_btn.clicked.connect(self._solve)
        self.zero_btn.clicked.connect(self._zero_current_pose)
        btns.addWidget(self.solve_btn)
        btns.addWidget(self.zero_btn)
        l.addLayout(btns)

        self.results_text = QPlainTextEdit()
        self.results_text.setReadOnly(True)
        l.addWidget(QLabel("Results"))
        l.addWidget(self.results_text)

        split.addWidget(left)

        fig = Figure(figsize=(6, 5))
        self.canvas = FigureCanvas(fig)
        self.ax_top = fig.add_subplot(211)
        self.ax_side = fig.add_subplot(212)
        split.addWidget(self.canvas)

        return w

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

        self.analysis_table = QTableWidget(0, 8)
        self.analysis_table.setHorizontalHeaderLabels(["Pitch", "Roll", "qL", "qR", "RMS", "Valid", "dFL", "dFR"])
        layout.addWidget(self.analysis_table)

        hfig = Figure(figsize=(6, 4))
        self.heat_canvas = FigureCanvas(hfig)
        self.ax_heat = hfig.add_subplot(111)
        layout.addWidget(self.heat_canvas)
        return w

    def _build_live_tab(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)

        io_box = QGroupBox("Live I/O")
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
        self.out_template = QLineEdit("L={left_counts},R={right_counts}\n")

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

        self.apply_live_btn = QPushButton("Apply Live I/O")
        self.apply_live_btn.clicked.connect(self._apply_live_io)
        grid.addWidget(self.apply_live_btn, 7, 0, 1, 2)

        layout.addWidget(io_box)

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
        )
        cfg.limits.cable_min = self.cable_min_edit.value()
        cfg.limits.cable_max = self.cable_max_edit.value()
        cfg.limits.motor_min_rad = self.motor_min_edit.value()
        cfg.limits.motor_max_rad = self.motor_max_edit.value()
        cfg.limits.rms_error_max = self.rms_max_edit.value()
        cfg.validate()
        return cfg

    def _apply_cfg_to_inputs(self) -> None:
        self._set_anchor_editor(self.moving_editor, self.cfg.moving_anchors)
        self._set_anchor_editor(self.fixed_editor, self.cfg.fixed_points)
        self.center_editor.x_edit.setText(str(self.cfg.rotation_center.x))
        self.center_editor.y_edit.setText(str(self.cfg.rotation_center.y))
        self.center_editor.z_edit.setText(str(self.cfg.rotation_center.z))
        self._set_scalar_editor(self.spool_radius_editor, self.cfg.spool_radii)
        self._set_scalar_editor(self.winding_editor, self.cfg.winding_signs)
        self.counts_per_out_edit.setText(str(self.cfg.counts_per_output_rev or 0))
        self.cpr_edit.setText(str(self.cfg.motor_encoder_cpr or 0))
        self.gear_edit.setText(str(self.cfg.gearbox_ratio or 0))
        self.cable_min_edit.setText(str(self.cfg.limits.cable_min))
        self.cable_max_edit.setText(str(self.cfg.limits.cable_max))
        self.motor_min_edit.setText(str(self.cfg.limits.motor_min_rad))
        self.motor_max_edit.setText(str(self.cfg.limits.motor_max_rad))
        self.rms_max_edit.setText(str(self.cfg.limits.rms_error_max))

    def _save_config(self) -> None:
        try:
            cfg = self._read_cfg_from_inputs()
            path, _ = QFileDialog.getSaveFileName(self, "Save config", "", "JSON (*.json)")
            if path:
                save_config(path, cfg)
                self.statusBar().showMessage(f"Saved {path}")
                self.cfg = cfg
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
            text = f"Asymmetric score: {out.asymmetric_score:.6f}\n"
            if out.warnings:
                text += "\n".join(out.warnings)
            else:
                text += "No geometry warnings"
            self.geometry_warnings.setPlainText(text)
        except Exception as exc:
            QMessageBox.critical(self, "Geometry analysis failed", str(exc))

    def _zero_current_pose(self) -> None:
        try:
            self.cfg = self._read_cfg_from_inputs()
            l0 = neutral_lengths(self.cfg)
            for k, edit in self.current_len_editor.edits.items():
                edit.setText(f"{l0[k]:.6f}")
            self.pitch_edit.setText("0")
            self.roll_edit.setText("0")
            self.statusBar().showMessage("Current pose zeroed to neutral")
        except Exception as exc:
            QMessageBox.critical(self, "Zero failed", str(exc))

    def _solve(self) -> None:
        try:
            self.cfg = self._read_cfg_from_inputs()
            mode = self.solve_mode.currentText()
            lengths = self.current_len_editor.float_values()
            if mode == "Manual Cable Solver":
                pose = estimate_pose_from_cable_lengths(self.cfg, lengths)
                self.pitch_edit.setText(f"{pose.pitch_deg:.5f}")
                self.roll_edit.setText(f"{pose.roll_deg:.5f}")
                msg = f"Estimated pose: P={pose.pitch_deg:.4f} deg, R={pose.roll_deg:.4f} deg, residual RMS={pose.residual_rms:.6f} m"
                if not pose.valid:
                    msg += "\nMeasured lengths are likely physically inconsistent"
                self.results_text.setPlainText(msg)
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
        except Exception as exc:
            QMessageBox.critical(self, "Solve failed", str(exc))

    def _show_solve_result(self, res) -> None:
        lines = []
        lines.append("Neutral lengths [m]: " + ", ".join(f"{k}={res.neutral_lengths[k]:.5f}" for k in CABLES))
        lines.append("Target lengths [m]: " + ", ".join(f"{k}={res.target_lengths[k]:.5f}" for k in CABLES))
        lines.append("Ideal deltas [m]: " + ", ".join(f"{k}={res.ideal_deltas[k]:+.6f}" for k in CABLES))
        lines.append(f"qL={res.q_left_rad:.6f} rad ({res.q_left_rad*57.2957795:.3f} deg)")
        lines.append(f"qR={res.q_right_rad:.6f} rad ({res.q_right_rad*57.2957795:.3f} deg)")
        lines.append("Pred deltas [m]: " + ", ".join(f"{k}={res.predicted_deltas[k]:+.6f}" for k in CABLES))
        lines.append("Residuals [m]: " + ", ".join(f"{k}={res.residuals[k]:+.6f}" for k in CABLES))
        lines.append(f"RMS error [m]: {res.rms_error:.6f}")
        lines.append(f"Target counts: L={res.target_counts_left}, R={res.target_counts_right}")
        lines.append(f"Clamped counts: L={res.clamped_counts_left}, R={res.clamped_counts_right}")
        if res.warnings:
            lines.append("Warnings:")
            lines.extend(f"- {w}" for w in res.warnings)
        self.results_text.setPlainText("\n".join(lines))

        ok = {k: (self.cfg.limits.cable_min <= res.target_lengths[k] <= self.cfg.limits.cable_max) for k in CABLES}
        plot_top_view(self.ax_top, self.cfg, radians(self.roll_edit.value()), radians(self.pitch_edit.value()), ok)
        plot_side_view(self.ax_side, self.cfg, radians(self.roll_edit.value()), radians(self.pitch_edit.value()), ok)
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
            self.analysis_summary.setPlainText(
                f"max RMS: {result.max_rms:.6f} m\n"
                f"mean RMS: {result.mean_rms:.6f} m\n"
                f"worst pose (pitch,roll): {result.worst_pose}\n"
                f"valid poses: {result.valid_pct:.2f}%\n"
                f"cable ranges: {result.cable_length_ranges}\n"
                f"motor ranges: {result.motor_angle_range}"
            )
            self.analysis_table.setRowCount(min(500, len(result.rows)))
            for i, row in enumerate(result.rows[:500]):
                for col, val in enumerate([
                    f"{row.pitch_deg:.2f}",
                    f"{row.roll_deg:.2f}",
                    f"{row.q_left_rad:.4f}",
                    f"{row.q_right_rad:.4f}",
                    f"{row.rms_error:.6f}",
                    "Y" if row.valid else "N",
                    f"{row.ideal_deltas['FL']:+.5f}",
                    f"{row.ideal_deltas['FR']:+.5f}",
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
        except Exception as exc:
            QMessageBox.critical(self, "Sweep failed", str(exc))

    def _refresh_ports(self) -> None:
        self.serial_port.clear()
        self.serial_port.addItems(list_serial_ports() or ["(none)"])

    def _apply_live_io(self) -> None:
        if self.live_source is not None:
            self.live_source.close()
            self.live_source = None
        if self.output is not None:
            self.output.close()
            self.output = None

        try:
            mode = self.input_mode.currentText()
            if mode == "serial":
                p = self.serial_port.currentText()
                if p and p != "(none)":
                    src = SerialInput(port=p, baud=self.serial_baud.value())
                    src.open()
                    self.live_source = src
            elif mode == "udp":
                src = UDPInput(host=self.udp_host.text().strip() or "0.0.0.0", port=self.udp_port.value())
                src.open()
                self.live_source = src

            out_mode = self.output_mode.currentText()
            if out_mode == "serial":
                p = self.serial_port.currentText()
                if p and p != "(none)":
                    self.output = SerialOutput(port=p, baud=self.serial_baud.value())
                    self.output.open()

            self.statusBar().showMessage("Live I/O applied")
        except Exception as exc:
            QMessageBox.critical(self, "Live I/O error", str(exc))

    def _poll_live_input(self) -> None:
        if not self.use_live_input.isChecked() or self.live_source is None:
            return
        try:
            line = self.live_source.poll_line()
            if not line:
                return
            parsed = parse_pitch_roll_line(line)
            if not parsed:
                return
            pitch, roll = parsed
            self.pitch_edit.setText(f"{pitch:.6f}")
            self.roll_edit.setText(f"{roll:.6f}")
            self._solve()
            self.live_log.appendPlainText(line)
        except Exception as exc:
            self.live_log.appendPlainText(f"Live input error: {exc}")

    def _send_output_if_enabled(self, res) -> None:
        if res.target_counts_left is None or res.target_counts_right is None:
            return
        mode = self.output_mode.currentText()
        if mode == "disabled":
            return
        if mode == "csv":
            payload = format_csv(res.target_counts_left, res.target_counts_right)
            self.live_log.appendPlainText(payload.strip())
            return
        if mode == "serial" and self.output is not None:
            template_text = self.out_template.text()
            try:
                template = codecs.decode(template_text, "unicode_escape")
            except Exception:
                template = template_text
                self.statusBar().showMessage("Output template escape decoding failed; using raw template text")
            payload = format_output(template, res.target_counts_left, res.target_counts_right)
            self.output.send_line(payload)
            self.live_log.appendPlainText(payload.strip())
