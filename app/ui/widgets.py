from __future__ import annotations

from typing import Dict

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

    def set_value(self, value: float) -> None:
        self.setText(f"{value:.6g}")


class IntEdit(QLineEdit):
    def __init__(self, value: int = 0, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setValidator(QIntValidator(-10_000_000, 10_000_000, self))
        self.setText(str(value))

    def value(self) -> int:
        text = self.text().strip()
        return int(text) if text else 0

    def set_value(self, value: int) -> None:
        self.setText(str(value))


class Vec3Editor(QWidget):
    def __init__(self, value: Vec3, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.x_edit = FloatEdit(value.x)
        self.y_edit = FloatEdit(value.y)
        self.z_edit = FloatEdit(value.z)
        layout.addWidget(QLabel("x"))
        layout.addWidget(self.x_edit)
        layout.addWidget(QLabel("y"))
        layout.addWidget(self.y_edit)
        layout.addWidget(QLabel("z"))
        layout.addWidget(self.z_edit)

    def value(self) -> Vec3:
        return Vec3(self.x_edit.value(), self.y_edit.value(), self.z_edit.value())

    def set_value(self, value: Vec3) -> None:
        self.x_edit.set_value(value.x)
        self.y_edit.set_value(value.y)
        self.z_edit.set_value(value.z)


class CableScalarEditor(QGroupBox):
    def __init__(self, title: str, defaults: Dict[str, float | int], use_integer_editor: bool = False, parent: QWidget | None = None) -> None:
        super().__init__(title, parent)
        grid = QGridLayout(self)
        self.edits: Dict[str, QLineEdit] = {}
        for row, cable in enumerate(CABLES):
            grid.addWidget(QLabel(cable), row, 0)
            edit: QLineEdit = IntEdit(int(defaults.get(cable, 0))) if use_integer_editor else FloatEdit(float(defaults.get(cable, 0.0)))
            self.edits[cable] = edit
            grid.addWidget(edit, row, 1)

    def float_values(self) -> Dict[str, float]:
        return {key: float(edit.text() or "0") for key, edit in self.edits.items()}

    def int_values(self) -> Dict[str, int]:
        return {key: int(edit.text() or "0") for key, edit in self.edits.items()}

    def set_values(self, values: Dict[str, float | int]) -> None:
        for key, value in values.items():
            edit = self.edits[key]
            edit.setText(str(value))


class AnchorEditor(QGroupBox):
    def __init__(self, title: str, defaults: Dict[str, Vec3], parent: QWidget | None = None) -> None:
        super().__init__(title, parent)
        self.editors: Dict[str, Vec3Editor] = {}
        form = QFormLayout(self)
        for cable in CABLES:
            editor = Vec3Editor(defaults[cable])
            self.editors[cable] = editor
            form.addRow(f"{cable} [m]", editor)

    def values(self) -> Dict[str, Vec3]:
        return {key: editor.value() for key, editor in self.editors.items()}

    def set_values(self, values: Dict[str, Vec3]) -> None:
        for key, value in values.items():
            self.editors[key].set_value(value)
