# Die einzelnen Bildschirmseiten. Jede Seite ist eine Funktion (draw, state) -> None und zeichnet
# auf ein 128x64-Bild im Modus "1". Welche Seiten in welcher Reihenfolge erscheinen, steht in STATUS_PAGES.
# Bedienung per 5-Wege-Joystick: links/rechts blaettert; jede Taste quittiert einen Alarm.

import math
from collections.abc import Callable

from PIL import Image, ImageDraw

from .state import Group, Motor, RobotState, TeachMode
from .widgets import (
    BIG,
    NORMAL,
    SMALL,
    H,
    W,
    Draw,
    bar,
    header,
    label_value,
    page_dots,
    text_center,
    text_right,
)

Page = Callable[[Draw, RobotState], None]

# Spielfeld HSL KidSize in Metern (Laenge x Breite), Mittelkreis-Radius
FIELD_L, FIELD_W, CIRCLE_R = 9.0, 6.0, 0.75
# Grenze fuer Drehmoment-Ueberlast in livelybot_serial (diagnostics.torque_overload_threshold)
OVERLOAD_NM = 10.0
LOAD_SCALE_NM = 15.0
MAX_IFACES = 4  # Interfaces auf der Netzseite
GROUP_LABELS: dict[Group, str] = {"left_leg": "L BEIN", "right_leg": "R BEIN", "arms": "ARME", "head": "KOPF"}

TEACH_LABELS: dict[TeachMode, str] = {
    TeachMode.OFF: "OFF",
    TeachMode.TEACH: "TEACH",
    TeachMode.HOLD: "HOLD",
}


# Akkuspannung oder "--.-V", wenn noch keine Nachricht kam
def volt(v: float | None) -> str:
    return "--.-V" if v is None else f"{v:.1f}V"


# Zahl oder "--", wenn der Wert noch unbekannt ist; fmt z. B. "{:.1f} A"
def _num(v: float | None, fmt: str = "{:.0f}") -> str:
    return "--" if v is None else fmt.format(v)


def _pct(v: float | None) -> str:
    return f"{_num(v)}%"


# Gestrichelte waagrechte Linie
def _dashed(draw: Draw, x0: int, x1: int, y: int, fill: int = 1, step: int = 3) -> None:
    for x in range(x0, x1 + 1, step):
        draw.point((x, y), fill=fill)


# Verlauf als Flaeche, neuester Wert rechts; lo..hi wird auf die Hoehe top..bottom abgebildet
def _area(draw: Draw, values: list[float], x0: int, x1: int, top: int, bottom: int, lo: float, hi: float) -> None:
    span = max(hi - lo, 1e-9)
    for i, v in enumerate(reversed(values)):
        x = x1 - i
        if x < x0:
            break
        y = bottom - round(max(0.0, min(1.0, (v - lo) / span)) * (bottom - top))
        draw.line((x, y, x, bottom), fill=1)


# Bytes pro Sekunde kurz: 850, 12K, 1.2M
def _rate(bps: float | None) -> str:
    if bps is None:
        return "--"
    if bps < 1000:
        return f"{bps:.0f}"
    if bps < 10_000:
        return f"{bps / 1000:.1f}K"
    if bps < 1_000_000:
        return f"{bps / 1000:.0f}K"
    return f"{bps / 1_000_000:.1f}M"


# ---------------------------------------------------------------- Statusseiten


# Seite 1: Rolle und aktuelles Verhalten
def page_behavior(draw: Draw, s: RobotState) -> None:
    header(draw, "STATUS", s)
    SMALL.draw(draw, (0, 17), "ROLLE")
    BIG.draw(draw, (26, 12), s.role)
    label_value(draw, 30, "AKT.", s.action)
    label_value(draw, 40, "GC", "verbunden" if s.gc_connected else "getrennt")
    SMALL.draw(draw, (0, 51), "LOK.")
    bar(draw, 26, 50, 70, 7, s.loc_percent)
    text_right(draw, W - 1, 50, _pct(s.loc_percent))


# Seite 2: Netzwerk. Die WLAN-IP steht schon in der Kopfzeile, hier Signal und alle Interfaces
def page_network(draw: Draw, s: RobotState) -> None:
    header(draw, "NETZ", s)
    label_value(draw, 12, "RSSI", f"{s.wifi_rssi} dBm" if s.wifi_rssi is not None else "getrennt")

    if not s.net_ifaces:
        NORMAL.draw(draw, (0, 24), "keine Interfaces")
        return
    for i, n in enumerate(s.net_ifaces[:MAX_IFACES]):
        y = 20 + i * 8
        SMALL.draw(draw, (1, y), n.name[:7])
        if not n.up:
            SMALL.draw(draw, (31, y), "AUS")
            continue
        SMALL.draw(draw, (31, y), n.ip or "-")
        text_right(draw, W - 1, y, f"{_rate(n.rx_bps)} {_rate(n.tx_bps)}", SMALL)
    SMALL.draw(draw, (1, 54), "IF  IPV4")
    text_right(draw, W - 1, 54, "EMPF SEND B/S", SMALL)


# Ein Motorkaestchen: ID im Umriss, von unten gefuellt je nach Drehmoment (LOAD_SCALE_NM = voll);
# die Ziffern werden ueber der Fuellung invertiert. Ausgefallene Motoren sind durchgestrichen
def _motor_cell(draw: Draw, m: Motor, x: int, y: int, w: int, h: int = 10) -> None:
    draw.rectangle((x, y, x + w - 1, y + h - 1), outline=1)
    if not m.ok:
        draw.line((x, y, x + w - 1, y + h - 1), fill=1)
        draw.line((x, y + h - 1, x + w - 1, y), fill=1)
        return
    label = Image.new("1", (w, h))
    text_center(ImageDraw.Draw(label), 3, str(m.id), SMALL, 1, 0, w)
    level = 0 if m.torque_nm is None else round(min(1.0, m.torque_nm / LOAD_SCALE_NM) * (h - 2))
    for py in range(1, h - 1):
        for px in range(1, w - 1):
            filled = py >= h - 1 - level
            draw.point((x + px, y + py), fill=int(filled != bool(label.getpixel((px, py)))))


def page_motors(draw: Draw, s: RobotState) -> None:
    header(draw, "MOTOREN", s)
    legs: list[tuple[str, Group]] = [("L", "left_leg"), ("R", "right_leg")]
    for row, (side, group) in enumerate(legs):
        y = 12 + row * 12
        SMALL.draw(draw, (0, y + 3), side)
        for col, m in enumerate(s.motors_of(group)):
            _motor_cell(draw, m, 9 + col * 20, y, 18)
    for col, m in enumerate(s.motors_of("head", "arms")[:10]):
        _motor_cell(draw, m, 4 + col * 12, 36, 10)
    bad, peak = s.bad_motors, s.peak_motor
    if bad:
        NORMAL.draw(draw, (0, 50), f"{bad[0].id} {bad[0].name}")
        text_right(draw, W - 1, 51, bad[0].error if len(bad) == 1 else f"+{len(bad) - 1} WEITERE", SMALL)
    elif not s.motors:
        NORMAL.draw(draw, (0, 50), "keine Motordaten")
    elif peak:
        SMALL.draw(draw, (0, 51), "MAX")
        NORMAL.draw(draw, (18, 50), f"{peak.torque_nm:.1f}Nm {peak.name}")
    else:
        NORMAL.draw(draw, (0, 50), f"alle {len(s.motors)} Motoren OK")


# Seite 4: Rechnerlast und Akku
def page_system(draw: Draw, s: RobotState) -> None:
    header(draw, "SYSTEM", s)
    SMALL.draw(draw, (0, 13), "CPU")
    bar(draw, 18, 12, 66, 7, s.cpu_percent)
    text_right(draw, W - 1, 12, f"{_pct(s.cpu_percent)} {_num(s.cpu_temp_c)}C")
    SMALL.draw(draw, (0, 23), "RAM")
    bar(draw, 18, 22, 66, 7, s.ram_percent)
    text_right(draw, W - 1, 22, _pct(s.ram_percent))
    b = s.battery
    SMALL.draw(draw, (0, 33), "BAT")
    bar(draw, 18, 32, 66, 7, b.percent)
    text_right(draw, W - 1, 32, _pct(b.percent))
    NORMAL.draw(draw, (0, 44), f"{volt(b.voltage_v)} {_num(b.current_a, '{:.1f}')}A {_num(b.power_w)}W {_num(b.temp_c)}C")

# Seite 0 (links von Seite 1): Teach-Mode per Knopfdruck umschalten
def page_teach(draw: Draw, s: RobotState) -> None:
    header(draw, "TEACH", s)
    text_center(draw, 12, "AKTUELLER MODUS", SMALL)
    text_center(draw, btn_y + 4, TEACH_LABELS[s.teach_mode], BIG)
    next_label = "HOLD" if s.teach_mode == TeachMode.TEACH else "TEACH"
    text_center(draw, 48, f"KLICK: {next_label}", NORMAL)

# ---------------------------------------------------------------- Graph-Seiten
# Fuers Debugging: Feldkarte, Last je Gelenk, Beinlast


# Seite FELD: Feldkarte mit Position, Blickrichtung und Spur, rechts die Zahlen
def page_field(draw: Draw, s: RobotState) -> None:
    header(draw, "FELD", s)
    ppm = 8  # Pixel pro Meter
    fw, fh = int(FIELD_L * ppm), int(FIELD_W * ppm)
    x0, y0 = 3, 12
    draw.rectangle((x0, y0, x0 + fw, y0 + fh), outline=1)
    cx, cy = x0 + fw // 2, y0 + fh // 2
    draw.line((cx, y0, cx, y0 + fh), fill=1)
    r = int(CIRCLE_R * ppm)
    draw.ellipse((cx - r, cy - r, cx + r, cy + r), outline=1)
    goal = int(2.6 * ppm) // 2
    draw.line((x0 - 2, cy - goal, x0 - 2, cy + goal), fill=1)  # eigenes Tor links
    draw.line(
        (x0 + fw + 2, cy - goal, x0 + fw + 2, cy + goal), fill=1
    )  # gegnerisches Tor rechts

    def to_px(x: float, y: float) -> tuple[int, int]:
        return round(cx + x * ppm), round(cy - y * ppm)

    for x, y in s.pose_trail[:-1]:
        draw.point(to_px(x, y), fill=1)
    tx = x0 + fw + 5
    if s.pose is None:
        SMALL.draw(draw, (tx, 14), "KEINE")
        SMALL.draw(draw, (tx, 21), "POSE")
        return
    x, y, theta = s.pose
    px, py = to_px(x, y)
    draw.ellipse((px - 2, py - 2, px + 2, py + 2), fill=1)
    a = math.radians(theta)
    draw.line(
        (px, py, px + round(6 * math.cos(a)), py - round(6 * math.sin(a))), fill=1
    )
    SMALL.draw(draw, (tx, 13), "X")
    text_right(draw, W - 1, 12, f"{x:+.1f}")
    SMALL.draw(draw, (tx, 22), "Y")
    text_right(draw, W - 1, 21, f"{y:+.1f}")
    SMALL.draw(draw, (tx, 31), "W")
    text_right(draw, W - 1, 30, f"{theta:+d}")
    SMALL.draw(draw, (tx, 41), "LOK")
    text_right(draw, W - 1, 40, _pct(s.loc_percent))
    bar(draw, tx, 49, W - tx, 6, s.loc_percent)


# Seite LAST: Drehmoment aller Motoren als Balken, gestrichelt die Ueberlastgrenze
def page_load(draw: Draw, s: RobotState) -> None:
    header(draw, "LAST", s)
    top, bottom = 12, 45
    height = bottom - top
    x = 1
    groups: list[tuple[Group, int, int]] = []  # Gruppe, x links, x rechts
    prev: Group | None = None
    for m in s.motors:
        g = m.group
        if g != prev:
            if prev is not None:
                x += 3
            groups.append((g, x, x))
            prev = g
        if not m.ok:
            draw.rectangle((x, top, x + 3, bottom), outline=1)
            draw.line((x, top, x + 3, bottom), fill=1)
        elif m.torque_nm is not None:
            h = round(min(1.0, m.torque_nm / LOAD_SCALE_NM) * height)
            draw.rectangle((x, bottom - h, x + 3, bottom), fill=1)
        draw.point((x + 1, bottom + 1), fill=1)
        groups[-1] = (g, groups[-1][1], x + 3)
        x += 5
    if s.motors:
        _dashed(draw, 0, x - 2, bottom - round(OVERLOAD_NM / LOAD_SCALE_NM * height))
    for g, gx0, gx1 in groups:
        label = GROUP_LABELS[g]
        label_w = SMALL.width(label)
        if label_w <= gx1 - gx0 + 4:
            SMALL.draw(draw, ((gx0 + gx1 - label_w) // 2, 48), label)
    peak = s.peak_motor
    if not s.motors:
        SMALL.draw(draw, (1, 54), "KEINE MOTORDATEN")
    elif peak:
        SMALL.draw(draw, (1, 54), f"MAX {peak.torque_nm:.1f} NM {peak.name}")


# Seite LASTVERLAUF: hoechste Bein-Last der letzten 6 s als Linie, mit Ueberlastgrenze
def page_load_history(draw: Draw, s: RobotState) -> None:
    header(draw, "BEINLAST", s)
    x0, x1, top, bottom = 14, W - 1, 12, 50
    height = bottom - top
    SMALL.draw(draw, (0, top), f"{LOAD_SCALE_NM:.0f}")
    SMALL.draw(
        draw,
        (0, bottom - round(OVERLOAD_NM / LOAD_SCALE_NM * height) - 2),
        f"{OVERLOAD_NM:.0f}",
    )
    SMALL.draw(draw, (4, bottom - 4), "0")
    draw.line((x0 - 2, top, x0 - 2, bottom), fill=1)
    draw.line((x0 - 2, bottom, x1, bottom), fill=1)
    _dashed(draw, x0, x1, bottom - round(OVERLOAD_NM / LOAD_SCALE_NM * height))
    hist = s.leg_torque_history
    step = (x1 - x0) / 59  # 60 Werte = 6 s
    pts = []
    for i, v in enumerate(hist):
        x = x1 - round((len(hist) - 1 - i) * step)
        y = bottom - round(min(1.0, v / LOAD_SCALE_NM) * height)
        pts.append((x, y))
    if len(pts) > 1:
        draw.line(pts, fill=1)
    elif pts:
        draw.point(pts[0], fill=1)
    SMALL.draw(draw, (x0, 54), "-6 S")
    if hist:
        text_right(draw, W - 1, 53, f"{hist[-1]:.1f} NM", SMALL)
    else:
        text_right(draw, W - 1, 53, "KEINE DATEN", SMALL)


STATUS_PAGES: list[Page] = [
    page_teach,
    page_behavior,
    page_network,
    page_motors,
    page_system,
    page_field,
    page_load,
    page_load_history,
]


# Rendert Statusseite index inkl. Seitenpunkten
def render_status(state: RobotState, index: int) -> Image.Image:
    img = Image.new("1", (W, H), 0)
    d = ImageDraw.Draw(img)
    STATUS_PAGES[index](d, state)
    page_dots(d, index, len(STATUS_PAGES))
    return img



# Alarm: ganzer Bildschirm invertiert, damit er aus Entfernung auffaellt.
# Im Betrieb im Sekundentakt mit der Normalansicht abwechselnd blinken lassen.
def render_alarm(title: str = "FEHLER", lines: tuple[str, ...] = ("Motor 4  L-Knie", "keine Antwort")) -> Image.Image:
    img = Image.new("1", (W, H), 1)
    d = ImageDraw.Draw(img)
    d.rectangle((2, 2, W - 3, H - 3), outline=0)
    text_center(d, 6, title, BIG, fill=0)
    for i, line in enumerate(lines[:2]):
        text_center(d, 25 + i * 10, line, NORMAL, fill=0)
    return img
