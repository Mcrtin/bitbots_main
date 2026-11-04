# Spielt ein rclpy-Log mit "msg=<repr>"-Zeilen (z. B. node_output.txt aus display.py) in RosStatusSource ab.
# So lassen sich die Seiten mit echten Daten testen, ohne ROS. Die Nachrichten werden mit ast gelesen, nicht mit eval.
# Aufruf: python -m pi_plus_display.log_replay node_output.txt ausgabe_ordner

import argparse
import ast
import re
from pathlib import Path
from types import SimpleNamespace

from .ros_source import RosStatusSource

# Eintrag: "[INFO] [my_node]: <text> (<callback>() at <datei>:<zeile>)", Text kann ueber mehrere Zeilen gehen
_ENTRY = re.compile(r"^\[(\w+)\] \[([\w/]+)\]: (.*) \((\w+)\(\) at [^)]*:\d+\)\s*$", re.S)
_ENTRY_START = re.compile(r"\n(?=\[(?:DEBUG|INFO|WARN|ERROR|FATAL)\] \[)")

# Nachrichtentyp -> Callback der Quelle; PoseWithCovarianceStamped haengt vom Callback-Namen ab (Ball oder Pose)
HANDLERS = {
    "diagnostic_msgs.msg.DiagnosticArray": "on_diagnostics",
    "sensor_msgs.msg.JointState": "on_joint_states",
    "bitbots_msgs.msg.Strategy": "on_strategy",
    "sensor_msgs.msg.CameraInfo": "on_camera_info",
    "std_msgs.msg.Float32": "on_battery",
    "sensor_msgs.msg.Imu": "on_imu",
    "game_controller_hsl_interfaces.msg.GameState": "on_gamestate",
    "bitbots_msgs.msg.RobotControlState": "on_robot_state",
    "bitbots_msgs.msg.TeamData": "on_team_data",
    "bitbots_msgs.msg.Workload": "on_workload",
}


class LogMsg(SimpleNamespace):
    msg_type = ""


# Nachbau einer Nachricht aus ihrem repr: Aufrufe werden zu LogMsg, array(...) zu Listen
def _build(node):
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.List | ast.Tuple):
        return [_build(e) for e in node.elts]
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub | ast.UAdd):
        value = _build(node.operand)
        return -value if isinstance(node.op, ast.USub) else value
    if isinstance(node, ast.Call):
        name = ast.unparse(node.func)
        if name == "array":
            return _build(node.args[0])
        msg = LogMsg(**{k.arg: _build(k.value) for k in node.keywords if k.arg != "dtype"})
        msg.msg_type = name
        return msg
    raise ValueError(f"unerwarteter Ausdruck im Log: {ast.unparse(node)[:80]}")


# Liefert (callback, nachricht) fuer jeden Logeintrag mit "msg=..."
def read_log(path):
    text = Path(path).read_text()
    for chunk in _ENTRY_START.split(text):
        m = _ENTRY.match(chunk.strip())
        if not m or not m.group(3).startswith("msg="):
            continue
        yield m.group(4), _build(ast.parse(m.group(3)[4:], mode="eval").body)


def _stamp(msg):
    header = getattr(msg, "header", None)
    if header is None:
        return None
    return header.stamp.sec + header.stamp.nanosec * 1e-9


class LogReplay:
    # Ordnet jeder Nachricht eine Zeit zu (header.stamp, sonst die zuletzt gesehene) und spielt sie zeitrichtig ab
    def __init__(self, path, robot_name="SIM", player_number=None):
        self.events = []
        t = 0.0
        for callback, msg in read_log(path):
            handler = HANDLERS.get(msg.msg_type)
            if msg.msg_type == "geometry_msgs.msg.PoseWithCovarianceStamped":
                handler = "on_ball" if "ball" in callback else "on_pose"
            if handler is None:
                continue
            stamp = _stamp(msg)
            if stamp is not None and stamp > 0:
                t = max(t, stamp)
            self.events.append((t, handler, msg))
        if not self.events:
            raise ValueError(f"{path}: keine lesbaren Nachrichten")
        self.start = self.events[0][0]
        self.end = self.events[-1][0]
        self.now = self.start
        self._next = 0
        self.source = RosStatusSource(player_number=player_number, clock=lambda: self.now)
        self.source._state.robot_name = robot_name

    # Alle Nachrichten bis Zeitpunkt t an die Quelle geben
    def advance_to(self, t):
        self.now = t
        while self._next < len(self.events) and self.events[self._next][0] <= t:
            _, handler, msg = self.events[self._next]
            getattr(self.source, handler)(msg)
            self._next += 1


def main():
    from PIL import Image

    from .pages import STATUS_PAGES, render_status

    ap = argparse.ArgumentParser(description="Seiten aus einem Node-Log rendern")
    ap.add_argument("log")
    ap.add_argument("out")
    ap.add_argument("--fps", type=float, default=10.0)
    ap.add_argument("--scale", type=int, default=4)
    args = ap.parse_args()

    replay = LogReplay(args.log)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    pages = [(f"{i + 1:02d}_{name}", lambda s, i=i: render_status(s, i)) for i, (name, _) in enumerate(STATUS_PAGES)]
    frames = {name: [] for name, _ in pages}
    t, dt = replay.start, 1.0 / args.fps
    while True:
        replay.advance_to(t)
        replay.source.update(dt)
        state = replay.source.snapshot()
        if state.battery_percent is not None:
            state.battery_percent = int(round(state.battery_percent))
        for name, render in pages:
            frames[name].append(render(state))
        if t >= replay.end:
            break
        t = min(t + dt, replay.end)

    def big(img):
        return img.convert("L").resize((img.width * args.scale, img.height * args.scale), Image.NEAREST)

    for name, imgs in frames.items():
        big(imgs[-1]).save(out / f"{name}.png")
        seq = [big(i) for i in imgs]
        seq[0].save(out / f"{name}.gif", save_all=True, append_images=seq[1:], duration=int(dt * 1000), loop=0)
    print(
        f"{len(frames)} Seiten, {len(next(iter(frames.values())))} Bilder je Seite, {replay.end - replay.start:.1f} s Log"
    )


if __name__ == "__main__":
    main()
