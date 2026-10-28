"""Light-theme Liquid Glass regressions: fields and sidebar use opaque fills with explicit text colour."""
from coldspray.gui import style


def test_light_fields_opaque_with_ink():
    qss = style.stylesheet("light")
    line = next(l for l in qss.splitlines() if l.startswith("QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox { background"))
    assert "rgba" not in line and "color: #" in line


def test_sidebar_scroll_area_opaque():
    qss = style.stylesheet("light")
    line = [l for l in qss.splitlines() if l.startswith("QScrollArea, QScrollArea > QWidget > QWidget")][-1]
    assert "rgba" not in line
