"""3Dconnexion SpaceMouse read directly over USB HID (no 3DxWare SDK needed).

The device reports only when something changes: translation (report 1),
rotation (report 2) and the button bitmask (report 3), each axis a signed
16-bit value of about +-350 at full deflection. Newer models send all six axes
in report 1. `poll()` drains the pending reports and returns the current state
in user terms:

    t = (right, forward/away from you, up)        -1..1
    r = (tilt forward, tilt right, twist anticlockwise seen from above)   -1..1
"""

import numpy as np

VENDORS = (0x256F, 0x046D)          # 3Dconnexion, Logitech (older 3Dconnexion devices)
FULL_SCALE = 350.0
DEADZONE = 0.06
# device axes (x right, y towards you, z down) -> user axes above. CHECK on the device.
T_SIGNS = np.array([1.0, -1.0, -1.0])
R_SIGNS = np.array([-1.0, 1.0, -1.0])


def _int16s(data, start, count) -> np.ndarray:
    raw = bytes(data[start:start + 2 * count])
    return np.frombuffer(raw, dtype="<i2").astype(float)


class SpaceMouse:
    def __init__(self, device, name: str):
        self.dev, self.name = device, name
        self.t, self.r = np.zeros(3), np.zeros(3)
        self.buttons = 0

    @classmethod
    def open(cls):
        """The first SpaceMouse found, or None (no device, or hidapi not installed)."""
        try:
            import hid
        except ImportError:
            return None
        for info in hid.enumerate():
            # usage page 1 / usage 8 = generic desktop multi-axis controller
            if info["vendor_id"] in VENDORS and info["usage_page"] == 1 and info["usage"] == 8:
                try:
                    dev = hid.device()
                    dev.open_path(info["path"])
                    dev.set_nonblocking(True)
                except OSError:
                    continue
                return cls(dev, info["product_string"] or "SpaceMouse")
        return None

    def poll(self) -> list[int]:
        """Read everything pending; returns the buttons newly pressed (0-based)."""
        pressed = []
        while True:
            try:
                data = self.dev.read(64)
            except OSError:          # unplugged
                data = None
            if not data:
                break
            kind = data[0]
            if kind == 1:
                self.t = _int16s(data, 1, 3)
                if len(data) >= 13:
                    self.r = _int16s(data, 7, 3)
            elif kind == 2:
                self.r = _int16s(data, 1, 3)
            elif kind == 3:
                state = int.from_bytes(bytes(data[1:5]), "little")
                pressed += [b for b in range(32) if state & ~self.buttons & (1 << b)]
                self.buttons = state
        return pressed

    def motion(self) -> tuple[np.ndarray, np.ndarray]:
        """(t, r) scaled to -1..1 with a small deadzone, in the user axes above."""
        def scale(v, signs):
            v = np.clip(v / FULL_SCALE, -1.0, 1.0) * signs
            return np.where(np.abs(v) < DEADZONE, 0.0, v)
        return scale(self.t, T_SIGNS), scale(self.r, R_SIGNS)

    def close(self):
        self.dev.close()
