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
    effort_nm: float | None = None  # Drehmoment aus joint_states, None = unbekannt


@dataclass
class Node:
    name: str
    ok: bool = True


# Temperaturfuehler des Rechners (hardware_interfaces.temperatures) fuer die Temperaturseite
@dataclass
class TempSensor:
    name: str
    celsius: float


# Netzwerk-Interface mit Datenrate fuer die Seite NETZ-IO
@dataclass
class NetIface:
    name: str
    up: bool = True
    ip: str | None = None  # None = keine IPv4-Adresse (z. B. can0)
    rx_bps: float | None = None  # Bytes pro Sekunde; None = erst ein Messwert
    tx_bps: float | None = None


# Empfangsstatistik eines Topics fuer die Datenfluss-Seite
@dataclass
class TopicStat:
    name: str
    hz: float | None = None  # None = erst eine Nachricht, Rate noch unbekannt
    age_s: float | None = None  # None = noch nie empfangen


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
    kickoff_own: bool | None = True  # None = unbekannt (Team-ID fehlt)
    penalty: str = ""  # "" = keine, sonst z. B. "PUSHING"
    penalty_seconds: int = 0
    gc_connected: bool = True
    gc_age_s: float = 0.2
    gc_diag: str = ""  # Meldung der Diagnose "/Robot/Other/Game Controller", z. B. "Connected"

    # Verhalten
    role: str = "STURM"
    action: str = "Ball anlaufen"
    ball_seen: bool = True
    ball_dist_m: float = 1.2
    loc_confidence: float | None = 0.78  # 0..1; None = noch keine Pose
    pose: tuple | None = (1.3, -0.8, 45)  # x [m], y [m], theta [Grad]; None = keine Lokalisierung
    pose_trail: list = field(default_factory=list)  # letzte Positionen (x, y) fuer die Feldkarte
    teammates_seen: int = 4
    teammates_total: int = 5

    # Netzwerk
    wlan_ip: str = "172.20.1.13"
    eth_ip: str = "192.168.1.10"
    ssid: str = "SPL_A"
    wifi_rssi: int | None = -58

    # Hardware
    battery_voltage: float | None = 23.4  # None = keine Nachricht von battery_voltage
    battery_percent: int | None = 72
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

    # Akku aus livelybot_power: battery_voltage, battery_current, battery_temperature, bms_error
    battery_current_a: float | None = 3.2
    battery_temp_c: float | None = 31.0
    bms_error: int | None = 0  # 0 = kein Fehler; None = noch keine Nachricht
    battery_age_s: float | None = 0.4  # seit der letzten Spannung; None = noch nie
    battery_history: list = field(default_factory=list)  # Spannung je Sekunde, aeltester zuerst

    # System
    cpu_percent: int | None = 63  # None = unbekannt
    cpu_temp_c: int | None = 71
    ram_percent: int | None = 48
    # Rechner, lokal ueber hardware_interfaces (/sys, /proc)
    cpu_mhz: float | None = 1800.0  # hoechster aktueller Kerntakt
    cpu_history: list = field(default_factory=list)  # CPU-Last [%] je Sekunde, aeltester zuerst
    load_avg: tuple | None = (1.21, 0.97, 0.80)  # 1, 5, 15 min
    uptime_s: float | None = 4980.0
    ram_used_gb: float | None = 1.9
    ram_total_gb: float | None = 3.8
    disk_percent: int | None = 43  # belegt auf /
    temps: list = field(
        default_factory=lambda: [TempSensor("CPU", 71.0), TempSensor("GPU", 64.0), TempSensor("NVME", 48.0)]
    )
    net_ifaces: list = field(
        default_factory=lambda: [
            NetIface("wlan0", True, "172.20.1.13", 12500.0, 3100.0),
            NetIface("eth0", True, "192.168.1.10", 850.0, 400.0),
            NetIface("can0", True, None, 182000.0, 96000.0),
        ]
    )
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

    # Verlauf der hoechsten Bein-Last [Nm], ein Wert pro Zehntelsekunde, aeltester zuerst
    leg_effort_history: list = field(default_factory=list)
    topics: list = field(default_factory=list)  # TopicStat

    # Hinweis fuer die Alarmseite
    alarm_title: str = ""
    alarm_lines: tuple = ()
