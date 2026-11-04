# Alle Daten, die das Display anzeigt, in einem Objekt.
# Im echten Betrieb fuellt ein ROS-Node diese Felder aus den Topics
# (GameController, Behavior, Diagnose, /battery_voltage, /imu/data, ...).

from dataclasses import dataclass, field

# Rollen, durch die das Menue blaettert
ROLES = ["STURM", "ABWEHR", "TOR", "UNTERST."]


@dataclass
class Motor:
    id: int
    name: str
    ok: bool = True
    temp_c: float | None = 35.0  # None = unbekannt
    error: str = ""  # kurzer Fehlertext, wenn nicht ok


@dataclass
class Node:
    name: str
    ok: bool = True


@dataclass
class RobotState:
    # Identitaet
    robot_name: str = "AMY"
    player_number: int = 3
    team_name: str = "BITBOTS"

    # GameController
    game_state: str = "PLAYING"  # INITIAL, READY, SET, PLAYING, FINISHED
    secondary_state: str = ""  # z. B. "PENALTY SHOOT", "TIMEOUT"
    half: int = 1
    seconds_remaining: int = 462
    score_own: int = 2
    score_opp: int = 1
    opp_name: str = "HTWK"
    kickoff_own: bool = True
    penalty: str = ""  # "" = keine, sonst z. B. "PUSHING"
    penalty_seconds: int = 0
    gc_connected: bool = True
    gc_age_s: float = 0.2

    # Verhalten
    role: str = "STURM"
    action: str = "Ball anlaufen"
    ball_seen: bool = True
    ball_dist_m: float = 1.2
    loc_confidence: float = 0.78
    pose: tuple = (1.3, -0.8, 45)  # x [m], y [m], theta [Grad]
    teammates_seen: int = 4
    teammates_total: int = 5

    # Netzwerk
    wlan_ip: str = "172.20.1.13"
    eth_ip: str = "192.168.1.10"
    ssid: str = "SPL_A"
    wifi_rssi: int | None = -58

    # Hardware
    battery_voltage: float = 23.4
    battery_percent: int = 72
    imu_ok: bool = True
    roll_deg: float = 1.2
    pitch_deg: float = -3.4
    motors: list = field(
        default_factory=lambda: [
            Motor(i + 1, n)
            for i, n in enumerate(
                [
                    "L-Huefte-G",
                    "L-Huefte-R",
                    "L-Huefte-N",
                    "L-Knie",
                    "L-Fuss-N",
                    "L-Fuss-R",
                    "R-Huefte-G",
                    "R-Huefte-R",
                    "R-Huefte-N",
                    "R-Knie",
                    "R-Fuss-N",
                    "R-Fuss-R",
                ]
            )
        ]
    )

    # System
    cpu_percent: int = 63
    cpu_temp_c: int = 71
    ram_percent: int = 48
    nodes: list = field(
        default_factory=lambda: [
            Node("vision"),
            Node("walking"),
            Node("behavior"),
            Node("locali."),
            Node("gc_cli"),
            Node("teamcom"),
        ]
    )

    # Hinweis fuer die Alarmseite
    alarm_title: str = ""
    alarm_lines: tuple = ()
