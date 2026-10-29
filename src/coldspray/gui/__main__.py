import threading

# OpenCascade and trimesh take ~5 s to import and are first needed when the
# test part loads; import them alongside the GUI libraries (13 s -> 7.6 s).
threading.Thread(target=lambda: (__import__("coldspray.cad.step_io"), __import__("trimesh")),
                 daemon=True).start()

from coldspray.gui.app import main  # noqa: E402

main()
