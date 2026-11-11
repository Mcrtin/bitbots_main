# Die einzelnen Bildschirmseiten. Jede Seite ist eine Funktion
# (draw, state, **kw) -> None und zeichnet auf ein 128x64-Bild im Modus "1".
# Gruppen: Statusseiten, Hardware-Seiten, Graph-Seiten fuers Debugging, danach Sonderseiten (Menue, Alarm, Start).
# Welche Statusseiten in welcher Reihenfolge erscheinen, steht in STATUS_PAGES.
# Bedienung per 5-Wege-Joystick:
#     links/rechts  vorige/naechste Statusseite
#     Mitte         Menue oeffnen, im Menue: auswaehlen
#     hoch/runter   im Menue blaettern
#     Alarm         erscheint automatisch, Mitte = quittieren

import math

from PIL import Image, ImageDraw

from .state import TempSensor
from .widgets import (
    BIG,
    NORMAL,
    SMALL,
    H,
    W,
    bar,
    header,
    label_value,
    page_dots,
    status_tag,
    text_center,
    text_right,
)

# Spielfeld HSL KidSize in Metern (Laenge x Breite), Mittelkreis-Radius
FIELD_L, FIELD_W, CIRCLE_R = 9.0, 6.0, 0.75
# Grenze fuer Drehmoment-Ueberlast in livelybot_serial (diagnostics.torque_overload_threshold)
OVERLOAD_NM = 10.0
LOAD_SCALE_NM = 15.0
TEMP_MIN_C, TEMP_MAX_C = 20.0, 100.0  # Bereich der Temperaturbalken
TEMP_HOT_C = 80.0  # ab hier wird die Zeile invertiert
BATTERY_STALE_S = 3.0
ROWS = 5  # Zeilen auf den Listenseiten


# Akkuspannung oder "--.-V", wenn noch keine Nachricht kam
def volt(v):
    return "--.-V" if v is None else f"{v:.1f}V"


# Zahl oder "--", wenn der Wert noch unbekannt ist; fmt z. B. "{:.1f} A"
def _num(v, fmt="{:.0f}"):
    return "--" if v is None else fmt.format(v)


def _pct(v):
    return f"{_num(v)}%"


def _mmss(seconds):
    seconds = max(0, int(seconds))
    return f"{seconds // 60:02d}:{seconds % 60:02d}"


def _who(state):
    return f"{state.robot_name} #{state.player_number}"


# Gestrichelte waagrechte Linie
def _dashed(draw, x0, x1, y, fill=1, step=3):
    for x in range(x0, x1 + 1, step):
        draw.point((x, y), fill=fill)


# Verlauf als Flaeche, neuester Wert rechts; lo..hi wird auf die Hoehe top..bottom abgebildet
def _area(draw, values, x0, x1, top, bottom, lo, hi):
    span = max(hi - lo, 1e-9)
    for i, v in enumerate(reversed(values)):
        x = x1 - i
        if x < x0:
            break
        y = bottom - round(max(0.0, min(1.0, (v - lo) / span)) * (bottom - top))
        draw.line((x, y, x, bottom), fill=1)


# 4980 s -> "1H23M", 59 s -> "59S"
def _uptime(seconds):
    if seconds is None:
        return "--"
    m = int(seconds) // 60
    if m < 1:
        return f"{int(seconds)}S"
    if m < 60:
        return f"{m}M"
    if m < 24 * 60:
        return f"{m // 60}H{m % 60:02d}M"
    return f"{m // (24 * 60)}D{m // 60 % 24}H"


# Bytes pro Sekunde kurz: 850, 12K, 1.2M
def _rate(bps):
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
def page_behavior(draw, s):
    header(draw, _who(s), s)
    SMALL.draw(draw, (0, 17), "ROLLE")
    BIG.draw(draw, (26, 12), s.role)
    label_value(draw, 30, "TUT", s.action, x_value=26)
    ball = f"{s.ball_dist_m:.1f} m" if s.ball_seen else "nicht gesehen"
    label_value(draw, 40, "BALL", ball, x_value=26)
    SMALL.draw(draw, (0, 51), "LOK.")
    bar(draw, 26, 50, 70, 7, s.loc_confidence or 0.0)
    text_right(
        draw,
        W - 1,
        50,
        _pct(None if s.loc_confidence is None else round(s.loc_confidence * 100)),
    )


# Seite 2: Netzwerk. IP sofort ablesbar, ohne Monitor oder Scan
def page_network(draw, s):
    header(draw, "NETZWERK", s)
    label_value(draw, 12, "WLAN", s.wlan_ip, x_value=26)
    rssi = f"{s.wifi_rssi} dBm" if s.wifi_rssi is not None else "getrennt"
    text_right(draw, W - 1, 12, rssi, SMALL)

    if not s.net_ifaces:
        return
    for i, n in enumerate(s.net_ifaces[:ROWS - 1]):
        y = 20 + i * 8
        SMALL.draw(draw, (1, y), n.name[:7])
        if not n.up:
            SMALL.draw(draw, (31, y), "AUS")
            continue
        SMALL.draw(draw, (31, y), n.ip or "-")
        text_right(draw, W - 1, y, f"{_rate(n.rx_bps)} {_rate(n.tx_bps)}", SMALL)
    SMALL.draw(draw, (1, 54), "IF  IPV4")
    text_right(draw, W - 1, 54, "EMPF SEND B/S", SMALL)


# Seite 3: Motoren als Raster (ID im Kaestchen, Fehler invertiert), IMU, Akku
def page_motors(draw, s):
    header(draw, "MOTOREN", s)
    cell_w, cell_h, x0 = 18, 10, 9
    for row, side in enumerate(("L", "R")):
        y = 12 + row * 12
        SMALL.draw(draw, (0, y + 3), side)
        for col in range(6):
            if row * 6 + col >= len(s.motors):
                break
            m = s.motors[row * 6 + col]
            x = x0 + col * (cell_w + 2)
            label = str(m.id)
            if m.ok:
                draw.rectangle((x, y, x + cell_w - 1, y + cell_h - 1), outline=1)
                text_center(draw, y + 3, label, SMALL, 1, x, x + cell_w)
            else:
                draw.rectangle((x, y, x + cell_w - 1, y + cell_h - 1), fill=1)
                text_center(draw, y + 3, label, SMALL, 0, x, x + cell_w)
    bad = [m for m in s.motors if not m.ok]
    if bad:
        NORMAL.draw(draw, (0, 38), f"{bad[0].id} {bad[0].name}")
        text_right(draw, W - 1, 39, bad[0].error or "KEINE ANTW.", SMALL)
    elif not s.motors:
        NORMAL.draw(draw, (0, 38), "keine Motordaten")
    elif all(m.temp_c is None for m in s.motors):
        loaded = [m for m in s.motors if m.effort_nm is not None]
        if loaded:
            top = max(loaded, key=lambda m: abs(m.effort_nm))
            SMALL.draw(draw, (0, 39), "MAX")
            NORMAL.draw(draw, (18, 38), f"{abs(top.effort_nm):.1f}Nm {top.name}")
        else:
            NORMAL.draw(draw, (0, 38), f"alle {len(s.motors)} Motoren OK")
    else:
        hot = max((m for m in s.motors if m.temp_c is not None), key=lambda m: m.temp_c)
        SMALL.draw(draw, (0, 39), "MAX")
        NORMAL.draw(draw, (18, 38), f"{hot.temp_c:.0f}C  {hot.name}")
    SMALL.draw(draw, (0, 51), "IMU")
    w = status_tag(draw, 16, 49, s.imu_ok)
    NORMAL.draw(draw, (16 + w + 3, 50), f"R{s.roll_deg:+.0f} P{s.pitch_deg:+.0f}")
    text_right(draw, W - 1, 50, volt(s.battery_voltage))


# Seite 4: Rechnerlast (TODO: takt, cur und max ram, battery_voltage + temp diagnostics hinzufügen)
def page_system(draw, s):
    header(draw, "SYSTEM", s)
    SMALL.draw(draw, (0, 13), "CPU")
    bar(draw, 18, 12, 66, 7, (s.cpu_percent or 0) / 100)
    text_right(draw, W - 1, 12, f"{_pct(s.cpu_percent)} {_num(s.cpu_temp_c)}C")
    SMALL.draw(draw, (0, 23), "RAM")
    bar(draw, 18, 22, 66, 7, (s.ram_percent or 0) / 100)
    text_right(draw, W - 1, 22, _pct(s.ram_percent))

    label_value(
        draw,
        30,
        "BAT",
        "--" if s.battery_voltage is None else s.battery_voltage,
        x_value=26,
    )
    w = (
        "-- W"
        if s.battery_voltage is None or s.battery_current_a is None
        else f"{s.battery_voltage * s.battery_current_a:.0f} W"
    )
    c = "" if s.battery_temp_c is None else f" {_num(s.battery_temp_c)}C"
    text_right(draw, W - 1, 30, w + c)


# ---------------------------------------------------------------- Graph-Seiten
# Fuers Debugging, entstanden aus den echten Node-Daten (node_output.txt): Feldkarte, Last je Gelenk, Beinlast, Datenfluss


# Seite FELD: Feldkarte mit Position, Blickrichtung und Spur, rechts die Zahlen
def page_field(draw, s):
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

    def to_px(x, y):
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
    lok = "--" if s.loc_confidence is None else round(s.loc_confidence * 100)
    text_right(draw, W - 1, 40, f"{lok}%")
    bar(draw, tx, 49, W - tx, 6, s.loc_confidence or 0.0)


# Gruppe eines Motors fuer die Lastseite anhand des Namens (l-hip-pitch, head-yaw, ...)
def _group(name):
    if name.startswith("head"):
        return "KOPF"
    if any(j in name for j in ("hip", "thigh", "calf", "ankle")):
        return "L BEIN" if name.startswith("l") else "R BEIN"
    return "ARME"


# Seite LAST: Drehmoment aller Motoren als Balken, gestrichelt die Ueberlastgrenze
def page_load(draw, s):
    header(draw, "LAST", s)
    top, bottom = 12, 45
    height = bottom - top
    x = 1
    groups = []
    prev = None
    for m in s.motors:
        g = _group(m.name)
        if g != prev:
            if prev is not None:
                x += 3
            groups.append([g, x, x])
            prev = g
        if not m.ok:
            draw.rectangle((x, top, x + 3, bottom), outline=1)
            draw.line((x, top, x + 3, bottom), fill=1)
        elif m.effort_nm is not None:
            h = round(min(1.0, abs(m.effort_nm) / LOAD_SCALE_NM) * height)
            draw.rectangle((x, bottom - h, x + 3, bottom), fill=1)
        draw.point((x + 1, bottom + 1), fill=1)
        groups[-1][2] = x + 3
        x += 5
    if s.motors:
        _dashed(draw, 0, x - 2, bottom - round(OVERLOAD_NM / LOAD_SCALE_NM * height))
    for g, gx0, gx1 in groups:
        label_w = SMALL.width(g)
        if label_w <= gx1 - gx0 + 4:
            SMALL.draw(draw, ((gx0 + gx1 - label_w) // 2, 48), g)
    known = [m for m in s.motors if m.effort_nm is not None]
    if not s.motors:
        SMALL.draw(draw, (1, 54), "KEINE MOTORDATEN")
    elif known:
        top_m = max(known, key=lambda m: abs(m.effort_nm))
        SMALL.draw(draw, (1, 54), f"MAX {abs(top_m.effort_nm):.1f} NM {top_m.name}")


# Seite LASTVERLAUF: hoechste Bein-Last der letzten 6 s als Linie, mit Ueberlastgrenze
def page_load_history(draw, s):
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
    hist = s.leg_effort_history
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


# Seite DATENFLUSS: Empfangsrate je Topic, fehlende Topics zusammengefasst
def page_topics(draw, s):
    header(draw, "DATENFLUSS", s)
    received = [t for t in s.topics if t.age_s is not None]
    missing = [t for t in s.topics if t.age_s is None]
    for i, t in enumerate(received[:5]):
        y = 12 + i * 8
        # Veraltet: laenger still als 3 Perioden, mindestens 1 s
        stale = t.age_s > (max(1.0, 3.0 / t.hz) if t.hz else 1.0)
        if stale:
            draw.rectangle((0, y - 1, W - 1, y + 5), fill=1)
        fill = 0 if stale else 1
        SMALL.draw(draw, (1, y), t.name, fill)
        if not t.hz:
            rate = "-- HZ"
        else:
            rate = f"{t.hz:.0f} HZ" if t.hz >= 1 else f"{t.hz:.1f} HZ"
        text_right(draw, 72, y, rate, SMALL, fill)
        if stale:
            text_right(draw, W - 2, y, f"VOR {t.age_s:.0f} S", SMALL, 0)
        elif t.hz:
            # Rate logarithmisch: 1 Hz kurz, 100 Hz fast voll
            frac = min(1.0, math.log10(max(t.hz, 1.0)) / 2.0)
            draw.rectangle((78, y, 78 + max(1, round(frac * 48)), y + 4), fill=1)
    if missing:
        names = " ".join(t.name for t in missing[:3])
        more = f" +{len(missing) - 3}" if len(missing) > 3 else ""
        SMALL.draw(draw, (1, 54), f"FEHLT: {names}{more}")


STATUS_PAGES = [
    ("verhalten", page_behavior),
    ("netzwerk", page_network),
    ("motoren", page_motors),
    ("system", page_system),
    ("feld", page_field),
    ("last", page_load),
    ("beinlast", page_load_history),
    ("datenfluss", page_topics),
]


# Rendert Statusseite index inkl. Seitenpunkten
def render_status(state, index):
    img = Image.new("1", (W, H), 0)
    d = ImageDraw.Draw(img)
    STATUS_PAGES[index][1](d, state)
    page_dots(d, index, len(STATUS_PAGES))
    return img



# Alarm: ganzer Bildschirm invertiert, damit er aus Entfernung auffaellt.
# Im Betrieb im Sekundentakt mit der Normalansicht abwechselnd blinken lassen.
def render_alarm(
    state,
    title="FEHLER",
    lines=("Motor 4  L-Knie", "keine Antwort, 3 s"),
    hint="MITTE = quittieren",
):
    img = Image.new("1", (W, H), 1)
    d = ImageDraw.Draw(img)
    d.rectangle((2, 2, W - 3, H - 3), outline=0)
    text_center(d, 6, title, BIG, fill=0)
    for i, line in enumerate(lines[:2]):
        text_center(d, 25 + i * 10, line, NORMAL, fill=0)
    d.line((6, 47, W - 7, 47), fill=0)
    text_center(d, 51, hint, SMALL, fill=0)
    return img

