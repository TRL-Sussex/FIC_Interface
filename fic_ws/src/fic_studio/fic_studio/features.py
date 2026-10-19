"""Optional parts a build of the Studio may leave out, and how the rest finds out.

The public repository ships without the VR headset station (``fleet/vr_station.py``, which the
headset app's relay runs) and its pairing (``fleet/vr_pairing.py``). Without them the Studio hides
Settings > VR and the headset pairing card and starts no background relay; the VR target is unchanged.
"""
import importlib.util


def headset():
    """True when this build has the VR headset support (the headset station and its pairing)."""
    try:
        return importlib.util.find_spec('fic_studio.fleet.vr_pairing') is not None
    except (ImportError, ValueError):
        return False
