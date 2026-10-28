"""Screenshot each step of the GUI: python tools/ui_screenshots.py OUTDIR [--theme light|dark]

Loads the built-in test_block, visits the five steps (generating a toolpath on Toolpath) and
--size WxH sets the window size, --scale sets QT_SCALE_FACTOR.
Saves stepN_<name>.png (window grab) plus stepN_<name>_3d.png (the 3D view alone).
With no display it falls back to the offscreen platform, where the 3D views may be blank."""
import argparse
import ctypes
import os
import sys
from pathlib import Path

NAMES = ["part", "position", "toolpath", "simulate", "export"]


def has_display():
    if sys.platform == "win32":
        return bool(ctypes.windll.user32.GetSystemMetrics(0))
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outdir")
    ap.add_argument("--theme", choices=["light", "dark"], default="light")
    ap.add_argument("--size", default="1600x1000", help="window size WxH")
    ap.add_argument("--scale", type=float, default=None, help="QT_SCALE_FACTOR, e.g. 1.5")
    args = ap.parse_args()
    w, h = (int(v) for v in args.size.lower().split("x"))
    if args.scale:
        os.environ["QT_SCALE_FACTOR"] = str(args.scale)
    out = Path(args.outdir)
    out.mkdir(parents=True, exist_ok=True)
    if "QT_QPA_PLATFORM" not in os.environ:
        os.environ["QT_QPA_PLATFORM"] = "offscreen"  # default: never open real windows
    if os.environ["QT_QPA_PLATFORM"] == "offscreen":
        print("offscreen: 3D views may be blank")

    from PySide6.QtCore import QSettings
    from PySide6.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    from coldspray.gui import plan_jobs
    from coldspray.gui.app import MainWindow
    win = MainWindow(QSettings(str(out / "ui.ini"), QSettings.IniFormat))
    win.set_theme(args.theme)
    win.resize(w, h)
    win.show()
    pump0 = [app.processEvents() for _ in range(20)]
    msh = win.minimumSizeHint()
    print(f"requested {w}x{h} scale={args.scale or 1} actual {win.width()}x{win.height()} "
          f"minimumSizeHint {msh.width()}x{msh.height()} minimumSize "
          f"{win.minimumWidth()}x{win.minimumHeight()}")

    def pump(n=20):
        for _ in range(n):
            app.processEvents()

    def save(i):
        pump()
        name = f"step{i + 1}_{NAMES[i]}"
        win.grab().save(str(out / f"{name}.png"))
        plotter = win.part_plotter if i == 0 else win.plotter
        try:
            plotter.screenshot(str(out / f"{name}_3d.png"))
        except Exception as e:
            print(f"{name}: 3D screenshot failed: {e}")
        print("saved", name)

    win._set_part("test_block")
    pump()
    for i in range(5):
        win.go_to(i)
        if i == 2:
            win.generate()
            plan_jobs.wait(win)
        save(i)
    win.close()
    (out / "ui.ini").unlink(missing_ok=True)


if __name__ == "__main__":
    main()
