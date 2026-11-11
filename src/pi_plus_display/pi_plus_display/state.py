# Alle Daten, die das Display anzeigt, in einem Objekt.
# Im echten Betrieb fuellt ein ROS-Node diese Felder aus den Topics
# (Behavior, Diagnose, /battery_voltage, /imu/data, ...); hier steht nur, was eine Seite oder ein Alarm liest.

from dataclasses import dataclass, field


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


# Temperaturfuehler des Rechners (hardware_interfaces.temperatures), nur in local_hardware
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

    # GameController: gamestate kommt an oder die Diagnose des GC-Clients meldet "Connected"
    gc_connected: bool = True

    # Verhalten
    role: str = "STURM"
    action: str = "Ball anlaufen"
    loc_confidence: float | None = 0.78  # 0..1; None = noch keine Pose
    pose: tuple | None = (1.3, -0.8, 45)  # x [m], y [m], theta [Grad]; None = keine Lokalisierung
    pose_trail: list = field(default_factory=list)  # letzte Positionen (x, y) fuer die Feldkarte

    # Netzwerk
    wlan_ip: str = "172.20.1.13"
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

    # System
    cpu_percent: int | None = 63  # None = unbekannt
    cpu_temp_c: int | None = 71
    ram_percent: int | None = 48
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
