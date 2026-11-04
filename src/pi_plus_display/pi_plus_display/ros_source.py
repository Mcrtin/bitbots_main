# Fuellt den RobotState aus den Topics des Bit-Bots-Stacks.
# Die Subscriptions werden an eine bestehende Node gehaengt (siehe status_display.py),
# es gibt keinen eigenen Thread und keinen eigenen Executor.

import copy
import math
import socket
import subprocess
import threading
import time

from transforms3d.euler import quat2euler

from .state import Motor, RobotState
from .state import Node as NodeStatus

DEFAULT_TOPICS = {
    # Schluessel: Topic                           Typ                                        Quelle im Stack
    "gamestate": "gamestate",  # game_controller_hsl_interfaces/GameState  game_controller_hsl
    "strategy": "strategy",  # bitbots_msgs/Strategy                     bitbots_blackboard
    "robot_state": "robot_state",  # bitbots_msgs/RobotControlState            bitbots_hcm
    "ball": "ball_position_relative_filtered",  # geometry_msgs/PoseWithCovarianceStamped  ball_filter
    "pose": "pose_with_covariance",  # geometry_msgs/PoseWithCovarianceStamped   Lokalisierung
    "battery": "battery_voltage",  # std_msgs/Float32                          livelybot_power (power_node)
    "imu": "imu/data",  # sensor_msgs/Imu                           yesense_imu
    "camera_info": "/zed/zed_node/rgb/camera_info",  # sensor_msgs/CameraInfo  zed_wrapper, nur als Lebenszeichen
    "workload": "system_workload",  # bitbots_msgs/Workload                     system_monitor
    "diagnostics": "diagnostics_agg",  # diagnostic_msgs/DiagnosticArray  Gruppen + Motoren (Servos/<joint>)
    "team_data": "team_data",  # bitbots_msgs/TeamData                     team_communication
}

ROLE_NAMES = {0: "?", 1: "PAUSE", 2: "ANDERE", 3: "STURM", 4: "UNTERST.", 5: "ABWEHR", 6: "TOR"}
ACTION_NAMES = {
    0: "-",
    1: "Positionieren",
    2: "Ball anlaufen",
    3: "Tor schiessen",
    4: "Warten",
    5: "Schiessen",
    6: "Ball suchen",
    7: "Lokalisieren",
    8: "Passiv",
}
# RobotControlState: Zustaende, die wichtiger sind als die Verhaltensaktion
ROBOT_STATE_TEXT = {
    1: "Faellt",
    2: "Gefallen",
    3: "Steht auf",
    5: "Startet",
    6: "Faehrt runter",
    7: "Strafe",
    11: "Motoren aus",
    12: "HCM aus",
    13: "Hardwarefehler",
    14: "Hochgehoben",
}
GAME_STATE_NAMES = {0: "INITIAL", 1: "READY", 2: "SET", 3: "PLAYING", 4: "FINISHED"}
GAME_PHASE_TEXT = {1: "ELFMETERSCHIESSEN", 2: "VERLAENGERUNG", 3: "TIMEOUT"}
SET_PLAY_TEXT = {
    1: "FREISTOSS DIREKT",
    2: "FREISTOSS INDIR.",
    3: "ELFMETER",
    4: "EINWURF",
    5: "ABSTOSS",
    6: "ECKE",
}
PENALTY_NAMES = {
    1: "POSITION",
    2: "MOTION SET",
    3: "MOTION STOP",
    4: "GAME STUCK",
    5: "INCAPABLE",
    6: "PICK UP",
    7: "BALL HOLD",
    8: "LEAVING",
    9: "ARMS/HANDS",
    10: "PUSHING",
    11: "VERWARNT",
    12: "PLATZVERWEIS",
    13: "AUSWECHSEL",
}

GC_TIMEOUT_S = 3.0
BALL_TIMEOUT_S = 2.0
IMU_TIMEOUT_S = 1.0
CAMERA_TIMEOUT_S = 1.0

# Reihenfolge im Motorraster: erst linkes, dann rechtes Bein von der Huefte zum Fuss,
# danach alle anderen Motoren (Arme, Kopf) alphabetisch. Namen wie in livelybot_serial/config/robot.yaml.
LEG_JOINTS = ["hip_pitch", "hip_roll", "thigh", "calf", "ankle_pitch", "ankle_roll"]
# Meldungen aus robot::motorDiagnostic (livelybot_serial) -> kurzer Text fuer die Anzeige
MOTOR_MESSAGES = {"No CAN data": "KEINE ANTW."}
TEAM_TIMEOUT_S = 5.0


class RosStatusSource:
    # Datenquelle fuer DisplayApp: start/stop/update/snapshot wie im Mockup (sources.py)

    name = "ros2"
    simulated = False

    # battery_*_v: Spannungen fuer 0 % und 100 %, an den Akku des Pi Plus anpassen
    def __init__(self, node, topics=None, team_id=None, player_number=None, battery_empty_v=21.0, battery_full_v=25.2):
        self.node = node
        self.topics = {**DEFAULT_TOPICS, **(topics or {})}
        self.team_id = team_id
        self.battery_empty_v = battery_empty_v
        self.battery_full_v = battery_full_v
        self._lock = threading.Lock()
        self._stamps = {}  # Topic-Schluessel -> Empfangszeit (monotonic)
        self._teammates = {}  # robot_id -> Empfangszeit
        self._diag_groups = []  # NodeStatus der Diagnose-Gruppen, ohne Kamera
        self._strategy_action = "-"
        self._robot_state_text = None

        s = self._state = RobotState()
        s.robot_name = socket.gethostname().split(".")[0].upper()[:8]
        if player_number is not None:
            s.player_number = player_number
        s.opp_name = "GEGNER"
        # Testwerte aus RobotState leeren, bis echte Nachrichten kommen. Die Seiten koennen noch
        # kein "unbekannt" darstellen, Zahlen stehen daher bis zur ersten Nachricht auf 0.
        s.action = "warte auf Daten"
        s.game_state = s.role = "-"
        s.seconds_remaining = s.score_own = s.score_opp = 0
        s.ball_seen, s.loc_confidence, s.teammates_seen = False, 0.0, 0
        s.battery_voltage, s.battery_percent = 0.0, 0
        s.roll_deg = s.pitch_deg = 0.0
        s.cpu_percent = s.cpu_temp_c = s.ram_percent = 0
        s.gc_connected = False
        s.wlan_ip = s.eth_ip = s.ssid = "-"
        s.wifi_rssi = None
        s.motors, s.nodes = [], []
        self._subscribe()

    def _subscribe(self):
        from bitbots_msgs.msg import RobotControlState, Strategy, TeamData, Workload
        from diagnostic_msgs.msg import DiagnosticArray
        from game_controller_hsl_interfaces.msg import GameState
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from sensor_msgs.msg import CameraInfo, Imu
        from std_msgs.msg import Float32

        t = self.topics
        sub = self.node.create_subscription
        sub(GameState, t["gamestate"], self.on_gamestate, 1)
        sub(Strategy, t["strategy"], self.on_strategy, 1)
        sub(RobotControlState, t["robot_state"], self.on_robot_state, 1)
        sub(PoseWithCovarianceStamped, t["ball"], self.on_ball, 1)
        sub(PoseWithCovarianceStamped, t["pose"], self.on_pose, 1)
        sub(Float32, t["battery"], self.on_battery, 1)
        sub(Imu, t["imu"], self.on_imu, 1)
        sub(CameraInfo, t["camera_info"], self.on_camera_info, 1)
        sub(Workload, t["workload"], self.on_workload, 1)
        sub(DiagnosticArray, t["diagnostics"], self.on_diagnostics, 1)
        sub(TeamData, t["team_data"], self.on_team_data, 1)

    # ------------------------------------------------------------- Schnittstelle fuer DisplayApp

    def start(self):
        pass

    def stop(self):
        pass

    def snapshot(self):
        with self._lock:
            return copy.deepcopy(self._state)

    # Aus dem Menue, wirkt bisher nur auf die Anzeige
    def set_role(self, role):
        with self._lock:
            self._state.role = role

    def set_player_number(self, number):
        with self._lock:
            self._state.player_number = number

    def sim_key(self, ch):
        pass

    # Aus Empfangszeiten abgeleitete Werte (verbunden, gesehen, ...)
    def update(self, dt):
        now = time.monotonic()
        with self._lock:
            s = self._state
            gc_age = now - self._stamps.get("gamestate", -1e9)
            s.gc_connected = gc_age < GC_TIMEOUT_S
            s.gc_age_s = min(gc_age, 99.9)
            s.ball_seen = now - self._stamps.get("ball", -1e9) < BALL_TIMEOUT_S
            s.imu_ok = now - self._stamps.get("imu", -1e9) < IMU_TIMEOUT_S
            camera_ok = now - self._stamps.get("camera_info", -1e9) < CAMERA_TIMEOUT_S
            s.nodes = self._diag_groups[:5] + [NodeStatus("zed", camera_ok)]
            s.teammates_seen = sum(1 for t in self._teammates.values() if now - t < TEAM_TIMEOUT_S)
            if s.penalty and s.penalty_seconds > 0:
                s.penalty_seconds = max(0, s.penalty_seconds - dt)

    def _touch(self, key):
        self._stamps[key] = time.monotonic()

    # ------------------------------------------------------------- Callbacks

    def on_gamestate(self, msg):
        with self._lock:
            s = self._state
            self._touch("gamestate")
            s.game_state = GAME_STATE_NAMES.get(msg.main_state, "?")
            s.half = 1 if msg.first_half else 2
            s.seconds_remaining = max(0, msg.seconds_remaining)
            s.score_own, s.score_opp = msg.own_score, msg.rival_score
            if self.team_id is not None:
                s.kickoff_own = msg.kicking_team == self.team_id
            s.secondary_state = GAME_PHASE_TEXT.get(msg.game_phase) or SET_PLAY_TEXT.get(msg.set_play, "")
            if msg.stopped:
                s.secondary_state = "SPIEL GESTOPPT"
            if msg.penalized:
                s.penalty = PENALTY_NAMES.get(msg.penalty, "STRAFE")
                s.penalty_seconds = msg.seconds_till_unpenalized
            else:
                s.penalty, s.penalty_seconds = "", 0
            if msg.players_per_team:
                s.teammates_total = max(1, msg.players_per_team - 1)

    def on_strategy(self, msg):
        with self._lock:
            self._state.role = ROLE_NAMES.get(msg.role, "?")
            self._strategy_action = ACTION_NAMES.get(msg.action, "-")
            self._apply_action()

    def on_robot_state(self, msg):
        with self._lock:
            self._robot_state_text = ROBOT_STATE_TEXT.get(msg.state)
            self._apply_action()

    def _apply_action(self):
        # Zustand der Hardware (gefallen, Motoren aus) geht vor der Verhaltensaktion
        self._state.action = self._robot_state_text or self._strategy_action

    def on_ball(self, msg):
        p = msg.pose.pose.position
        with self._lock:
            self._touch("ball")
            self._state.ball_dist_m = math.hypot(p.x, p.y)

    def on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        _, _, yaw = quat2euler((q.w, q.x, q.y, q.z))
        cov = msg.pose.covariance
        # Grobe Konfidenz aus der Positionsunsicherheit (1 = sicher). Heuristik, ggf. anpassen.
        sigma = math.sqrt(max(0.0, cov[0]) + max(0.0, cov[7]))
        with self._lock:
            self._state.pose = (p.x, p.y, round(math.degrees(yaw)))
            self._state.loc_confidence = math.exp(-sigma)

    def on_battery(self, msg):
        v = float(msg.data)
        span = self.battery_full_v - self.battery_empty_v
        with self._lock:
            self._state.battery_voltage = v
            self._state.battery_percent = max(0.0, min(100.0, (v - self.battery_empty_v) / span * 100))

    def on_imu(self, msg):
        q = msg.orientation
        roll, pitch, _ = quat2euler((q.w, q.x, q.y, q.z))
        with self._lock:
            self._touch("imu")
            self._state.roll_deg, self._state.pitch_deg = math.degrees(roll), math.degrees(pitch)

    def on_camera_info(self, msg):
        with self._lock:
            self._touch("camera_info")

    def on_workload(self, msg):
        wlan = eth = None
        for iface in msg.network_interfaces:
            ip = next((a for a in iface.ip_addresses if ":" not in a), None)
            if not ip:
                continue
            if iface.name.startswith("wl") and wlan is None:
                wlan = ip
            elif iface.name.startswith("e") and eth is None:
                eth = ip
        with self._lock:
            s = self._state
            s.cpu_percent = int(msg.cpu_usage_overall)
            if msg.memory_total:
                s.ram_percent = int(100 * msg.memory_used / msg.memory_total)
            s.wlan_ip = wlan or "-"
            s.eth_ip = eth or "-"

    # Gruppen des Diagnostic Aggregators und je Motor ein Eintrag unter .../Servos/<joint>.
    # Der Aggregator haengt alles unter einen Wurzelpfad ("/Robot", siehe bitbots_diagnostic/config).
    # Angezeigt werden die Gruppen eine Ebene darunter, z. B. "/Robot/Servos".
    def on_diagnostics(self, msg):
        if not msg.status:
            return
        depth = min(st.name.count("/") for st in msg.status)
        if sum(1 for st in msg.status if st.name.count("/") == depth) == 1:
            depth += 1  # einzelne Wurzel ueberspringen
        groups = [
            NodeStatus(st.name.rsplit("/", 1)[-1].strip().lower()[:7], _level(st) <= 1)
            for st in msg.status
            if st.name.count("/") == depth
        ]
        servos = [st for st in msg.status if "/Servos/" in st.name]
        motors = [_motor_from_status(st) for st in sorted(servos, key=_motor_order)]
        for i, m in enumerate(motors):
            m.id = i + 1
        with self._lock:
            self._diag_groups = groups
            if motors:
                self._state.motors = motors

    def on_team_data(self, msg):
        with self._lock:
            if msg.robot_id != self._state.player_number:
                self._teammates[msg.robot_id] = time.monotonic()

    # Werte, die es nicht als Topic gibt: CPU-Temperatur, WLAN-Name und -Signal
    def poll_local(self):
        temp = _read_cpu_temp()
        ssid, rssi = _read_wifi()
        with self._lock:
            if temp is not None:
                self._state.cpu_temp_c = temp
            self._state.ssid = ssid or "-"
            self._state.wifi_rssi = rssi


# DiagnosticStatus.level ist in rclpy ein bytes-Objekt der Laenge 1
def _level(status):
    lv = status.level
    return lv[0] if isinstance(lv, bytes | bytearray) else int(lv)


def _servo_name(status):
    return status.name.split("/Servos/", 1)[1].strip()


def _motor_order(status):
    name = _servo_name(status)
    for side, prefix in enumerate(("l_", "r_")):
        for i, joint in enumerate(LEG_JOINTS):
            if name.startswith(prefix + joint):
                return (side, i, name)
    return (2, 0, name)


# OK und WARN (z. B. Drehmoment-Ueberlast) zaehlen als laufend, ERROR und STALE nicht
def _motor_from_status(status):
    ok = _level(status) <= 1
    error = "" if ok else MOTOR_MESSAGES.get(status.message, status.message.upper()[:11])
    return Motor(0, _short_joint(_servo_name(status)), ok, temp_c=None, error=error)


# 'LKnee' / 'left_knee_joint' -> kurzer Anzeigename
def _short_joint(name):
    return name.replace("_joint", "").replace("_", "-")[:12]


def _read_cpu_temp():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) // 1000
    except (OSError, ValueError):
        return None


# (SSID, RSSI in dBm) ueber /proc/net/wireless und iwgetid, sonst (None, None)
def _read_wifi():
    rssi = None
    try:
        with open("/proc/net/wireless") as f:
            for line in f.readlines()[2:]:
                parts = line.split()
                rssi = int(float(parts[3].rstrip(".")))
                break
    except (OSError, ValueError, IndexError):
        pass
    try:
        ssid = subprocess.run(["iwgetid", "-r"], capture_output=True, text=True, timeout=1).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        ssid = None
    return ssid or None, rssi
