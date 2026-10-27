"""Generic process form widgets (off screen)."""

import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtWidgets import QApplication, QComboBox, QDoubleSpinBox, QLineEdit, QSpinBox

from coldspray.gui.steps.toolpath import box_differs, box_value, make_process_box
from coldspray.models import ProcessConfig, load_process

app = QApplication.instance() or QApplication([])


def test_every_field_round_trips_through_its_widget():
    proc = load_process()
    calls = []
    boxes = {n: make_process_box(n, info, getattr(proc, n), lambda *_: calls.append(1))
             for n, info in ProcessConfig.model_fields.items()}
    assert isinstance(boxes["infill_pattern"], QComboBox)
    assert isinstance(boxes["turn_plane"], QLineEdit)
    assert isinstance(boxes["contour_passes"], QSpinBox)
    assert isinstance(boxes["standoff_mm"], QDoubleSpinBox)
    assert ProcessConfig(**{n: box_value(b) for n, b in boxes.items()}) == proc
    assert not any(box_differs(b, getattr(proc, n)) for n, b in boxes.items())

    combo = boxes["infill_pattern"]
    combo.setCurrentIndex(combo.findData("spiral"))
    assert box_value(combo) == "spiral" and box_differs(combo, proc.infill_pattern)
    assert calls
