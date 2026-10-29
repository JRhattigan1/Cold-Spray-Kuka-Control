import os
import shutil

import numpy as np
import pytest

from coldspray import config_io
from coldspray.models import CONFIG_DIR, XYZABC, load_cell, load_tool
from coldspray.tool import cad_tool_frame, parse_pendant, tool_frame

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")


@pytest.mark.parametrize("text", [
    "{X 517.79,Y 214.24,Z 130.91,A 0,B 0,C 0}",
    "DECL FRAME TOOL_DATA[1]={X 517.79,Y 214.24,Z 130.91,A 0.0,B 0.0,C 0.0}",
    "{ C 0, Z 130.91 ,  Y 214.24,X 517.79 , A 0 ,B 0 }",
])
def test_parse_pendant(text):
    f = parse_pendant(text)
    assert (f.x, f.y, f.z, f.a, f.b, f.c) == (517.79, 214.24, 130.91, 0, 0, 0)


def test_parse_negative_and_bad():
    assert parse_pendant("{X -1.5,Y 2E1,Z 0,A -90,B 0,C 180}").a == -90
    with pytest.raises(ValueError):
        parse_pendant("nothing here")


@pytest.fixture
def cfg(tmp_path):
    for n in ("tool.yaml", "cell.yaml"):
        shutil.copy(CONFIG_DIR / n, tmp_path / n)
    return tmp_path


def test_roundtrip_and_tcp_per_nozzle(cfg):
    tool = load_tool(cfg / "tool.yaml")
    cad = tool_frame(tool)
    assert np.allclose(cad, cad_tool_frame(tool.gun))            # nothing taught: CAD
    taught = XYZABC(x=517.79, y=214.24, z=130.91)
    tool.gun.nozzles["radial_injector"].tcp = taught
    config_io.save_tools(tool, cfg / "tool.yaml")
    text = (cfg / "tool.yaml").read_text()
    assert "# Controller backup" in text and "tool_number: 1" in text
    back = load_tool(cfg / "tool.yaml")
    assert back.gun.nozzles["radial_injector"].tcp == taught
    assert np.allclose(tool_frame(back), cad)                    # central selected: still CAD
    back.gun.nozzle = "radial_injector"
    assert np.allclose(tool_frame(back)[:3, 3], [517.79, 214.24, 130.91])
    config_io.save_bases({2: XYZABC(x=1, c=5)}, cfg / "cell.yaml")
    config_io.save_bases({2: XYZABC(x=3)}, cfg / "cell.yaml")
    assert load_cell(cfg / "cell.yaml").bases_taught == {2: XYZABC(x=3)}


def test_dialog_offscreen(cfg):
    from PySide6.QtWidgets import QApplication, QLabel
    from coldspray.gui.cell_dialog import CellDialog
    app = QApplication.instance() or QApplication([])
    d = CellDialog(tool_path=cfg / "tool.yaml", cell_path=cfg / "cell.yaml")
    d.add_tab(QLabel("home"), "HOME")
    assert [d.tabs.tabText(i) for i in range(3)] == ["Tool", "Base", "HOME"]
    assert d.tool_tab.table.rowCount() == 2 and d.tool_tab.frame(0) is None
    d.tool_tab.table.setCurrentCell(0, 2)
    d.tool_tab.paste.setText("{X 517.79,Y 214.24,Z 130.91,A 0,B 0,C 0}")
    d.tool_tab._paste()
    assert d.save() is True
    assert load_tool(cfg / "tool.yaml").gun.current_nozzle.tcp.x == 517.79
    d.close()
    app.processEvents()
