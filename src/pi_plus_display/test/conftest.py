# display.py imports rclpy and the ROS message packages. Where they are missing (laptop, CI), empty
# stub modules step in so the node's callbacks can still be tested.

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
