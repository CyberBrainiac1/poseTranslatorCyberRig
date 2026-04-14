from __future__ import annotations

from typing import Dict, Tuple

from PySide6.QtGui import QDoubleValidator, QIntValidator
from PySide6.QtWidgets import QFormLayout, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QWidget

from app.models import CABLES, Vec3


class FloatEdit(QLineEdit):
    def __init__(self, value: float = 0.0, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setValidator(QDoubleValidator(-1e9, 1e9, 8, self))
        self.setText(f"{value:.6g}")

    def value(self) -> float:
        text = self.text().strip()
        return float(text) if text else 0.0


class IntEdit(QLineEdit):
    def __init__(self, value: int = 0, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setValidator(QIntValidator(-10_000_000, 10_000_000, self))
        self.setText(str(value))

    def value(self) -> int:
        text = self.text().strip()
        return int(text) if text else 0


class Vec3Editor(QWidget):
    def __init__(self, value: Vec3, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.x_edit = FloatEdit(value.x)
        self.y_edit = FloatEdit(value.y)
        self.z_edit = FloatEdit(value.z)
        lay.addWidget(QLabel("x"))
        lay.addWidget(self.x_edit)
        lay.addWidget(QLabel("y"))
        lay.addWidget(self.y_edit)
        lay.addWidget(QLabel("z"))
        lay.addWidget(self.z_edit)

    def value(self) -> Vec3:
        return Vec3(self.x_edit.value(), self.y_edit.value(), self.z_edit.value())


class CableScalarEditor(QGroupBox):
    def __init__(self, title: str, defaults: Dict[str, float], use_integer_editor: bool = False, parent: QWidget | None = None) -> None:
        super().__init__(title, parent)
        grid = QGridLayout(self)
        self.edits: Dict[str, QLineEdit] = {}
        for i, c in enumerate(CABLES):
            grid.addWidget(QLabel(c), i, 0)
            edit: QLineEdit = IntEdit(int(defaults.get(c, 0))) if use_integer_editor else FloatEdit(float(defaults.get(c, 0.0)))
            self.edits[c] = edit
            grid.addWidget(edit, i, 1)

    def float_values(self) -> Dict[str, float]:
        out: Dict[str, float] = {}
        for k, edit in self.edits.items():
            out[k] = float(edit.text() or "0")
        return out

    def int_values(self) -> Dict[str, int]:
        out: Dict[str, int] = {}
        for k, edit in self.edits.items():
            out[k] = int(edit.text() or "0")
        return out


class AnchorEditor(QGroupBox):
    def __init__(self, title: str, defaults: Dict[str, Vec3], parent: QWidget | None = None) -> None:
        super().__init__(title, parent)
        self.editors: Dict[str, Vec3Editor] = {}
        form = QFormLayout(self)
        for c in CABLES:
            e = Vec3Editor(defaults[c])
            self.editors[c] = e
            form.addRow(f"{c} [m]", e)

    def values(self) -> Dict[str, Vec3]:
        return {k: e.value() for k, e in self.editors.items()}
