"""Application look: Fusion style with the light / dark palettes from gui/style.py."""

from coldspray.gui import style


def apply(app, mode: str = "system") -> str:
    """Style `app`. `mode`: "system" (follow Windows), "light" or "dark". Returns the mode used."""
    return style.apply(app, mode)
