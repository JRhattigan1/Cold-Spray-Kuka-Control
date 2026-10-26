"""Cold spray toolpath window: five steps across the top.

  1 Part     - the part or assembly on its own: body roles, hidden bodies, start and base faces
  2 Position - place it on the faceplate, substrate plate, table tilt / rotation, gantry, gun roll
  3 Toolpath - strategy and process parameters, Generate
  4 Simulate - play it back, reach, gun angle and collision checks
  5 Export   - save the KRL program

Run with:  python -m coldspray.gui
"""

from coldspray.gui.app import MainWindow, main

__all__ = ["MainWindow", "main"]
