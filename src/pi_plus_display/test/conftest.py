# display.py importiert rclpy und die ROS-Nachrichtenpakete. Wo die fehlen (Laptop, CI), springen
# leere Ersatzmodule ein, damit die Callbacks der Node trotzdem getestet werden koennen.

import importlib.util
import sys
import types


def _stub(name):
    mod = types.ModuleType(name)
    mod.__path__ = []
    mod.__getattr__ = lambda attr: type(attr, (), {"__init__": lambda self, *a, **k: None})
    sys.modules[name] = mod


for top, subs in {
    "rclpy": ["node", "parameter", "clock"],
    "bitbots_msgs": ["msg"],
    "diagnostic_msgs": ["msg"],
    "game_controller_hsl_interfaces": ["msg"],
    "geometry_msgs": ["msg"],
    "sensor_msgs": ["msg"],
}.items():
    if importlib.util.find_spec(top) is None:
        _stub(top)
        for sub in subs:
            _stub(f"{top}.{sub}")
