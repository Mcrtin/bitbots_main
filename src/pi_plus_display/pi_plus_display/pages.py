# Die einzelnen Bildschirmseiten. Jede Seite ist eine Funktion
# (draw, state, **kw) -> None und zeichnet auf ein 128x64-Bild im Modus "1".
# Bedienung per 5-Wege-Joystick:
#     links/rechts  vorige/naechste Statusseite
#     Mitte         Menue oeffnen, im Menue: auswaehlen
#     hoch/runter   im Menue blaettern
#     Alarm         erscheint automatisch, Mitte = quittieren

from PIL import Image, ImageDraw

from .widgets import BIG, NORMAL, SMALL, H, W, bar, header, label_value, page_dots, status_tag, text_center, text_right


def _mmss(seconds):
    seconds = max(0, int(seconds))
    return f"{seconds // 60:02d}:{seconds % 60:02d}"


def _who(state):
    return f"{state.robot_name} #{state.player_number}"


# ---------------------------------------------------------------- Statusseiten


# Seite 1: Spiel. Das Wichtigste fuer Schiedsrichter und Team am Feldrand
def page_game(draw, s):
    header(draw, _who(s), s)
    text_center(draw, 12, s.game_state, BIG)
    text_center(draw, 29, f"{s.team_name} {s.score_own}:{s.score_opp} {s.opp_name}")
    SMALL.draw(draw, (0, 41), f"{s.half}. HZ")
    text_center(draw, 40, _mmss(s.seconds_remaining))
    text_right(draw, W - 1, 41, "ANSTOSS" if s.kickoff_own else "GEGNER", SMALL)
    if s.penalty:
        draw.rectangle((0, 50, W - 1, 58), fill=1)
        text_center(draw, 51, f"STRAFE {s.penalty} {_mmss(s.penalty_seconds)}", NORMAL, fill=0)
    elif not s.gc_connected:
        draw.rectangle((0, 50, W - 1, 58), outline=1)
        text_center(draw, 52, "KEIN GAMECONTROLLER", SMALL)
    else:
        text_center(draw, 52, s.secondary_state or "keine Strafe", SMALL)


# Seite 2: Rolle und aktuelles Verhalten
def page_behavior(draw, s):
    header(draw, _who(s), s)
    SMALL.draw(draw, (0, 17), "ROLLE")
    BIG.draw(draw, (26, 12), s.role)
    label_value(draw, 30, "TUT", s.action, x_value=26)
    ball = f"{s.ball_dist_m:.1f} m" if s.ball_seen else "nicht gesehen"
    label_value(draw, 40, "BALL", ball, x_value=26)
    SMALL.draw(draw, (0, 51), "LOK.")
    bar(draw, 26, 50, 70, 7, s.loc_confidence)
    text_right(draw, W - 1, 50, f"{round(s.loc_confidence * 100)}%")


# Seite 3: Netzwerk. IP sofort ablesbar, ohne Monitor oder Scan
def page_network(draw, s):
    header(draw, "NETZWERK", s)
    label_value(draw, 12, "WLAN", s.wlan_ip, x_value=26)
    rssi = f"{s.wifi_rssi} dBm" if s.wifi_rssi is not None else "getrennt"
    label_value(draw, 22, "SSID", s.ssid, x_value=26)
    text_right(draw, W - 1, 23, rssi, SMALL)
    label_value(draw, 32, "ETH", s.eth_ip, x_value=26)
    SMALL.draw(draw, (0, 43), "GC")
    w = status_tag(draw, 26, 41, s.gc_connected)
    if s.gc_connected:
        NORMAL.draw(draw, (26 + w + 4, 42), f"vor {s.gc_age_s:.1f} s")
    SMALL.draw(draw, (0, 53), "TEAM")
    NORMAL.draw(draw, (26, 52), f"{s.teammates_seen}/{s.teammates_total} Mitspieler")


# Seite 4: Motoren als Raster (ID im Kaestchen, Fehler invertiert), IMU, Akku
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
        NORMAL.draw(draw, (0, 38), f"alle {len(s.motors)} Motoren OK")
    else:
        hot = max((m for m in s.motors if m.temp_c is not None), key=lambda m: m.temp_c)
        SMALL.draw(draw, (0, 39), "MAX")
        NORMAL.draw(draw, (18, 38), f"{hot.temp_c:.0f}C  {hot.name}")
    SMALL.draw(draw, (0, 51), "IMU")
    w = status_tag(draw, 16, 49, s.imu_ok)
    NORMAL.draw(draw, (16 + w + 3, 50), f"R{s.roll_deg:+.0f} P{s.pitch_deg:+.0f}")
    text_right(draw, W - 1, 50, f"{s.battery_voltage:.1f}V")


# Seite 5: Rechnerlast und Zustand der ROS-Nodes
def page_system(draw, s):
    header(draw, "SYSTEM", s)
    SMALL.draw(draw, (0, 13), "CPU")
    bar(draw, 18, 12, 66, 7, s.cpu_percent / 100)
    text_right(draw, W - 1, 12, f"{s.cpu_percent}% {s.cpu_temp_c}C")
    SMALL.draw(draw, (0, 23), "RAM")
    bar(draw, 18, 22, 66, 7, s.ram_percent / 100)
    text_right(draw, W - 1, 22, f"{s.ram_percent}%")
    draw.line((0, 32, W - 1, 32), fill=1)
    col_w = 43
    for i, n in enumerate(s.nodes[:6]):
        x = (i % 3) * col_w
        y = 36 + (i // 3) * 11
        if n.ok:
            draw.rectangle((x + 1, y + 2, x + 3, y + 4), fill=1)
            SMALL.draw(draw, (x + 6, y + 1), n.name)
        else:
            draw.rectangle((x, y - 1, x + col_w - 3, y + 7), fill=1)
            SMALL.draw(draw, (x + 2, y + 1), n.name + " !", 0)


STATUS_PAGES = [
    ("spiel", page_game),
    ("verhalten", page_behavior),
    ("netzwerk", page_network),
    ("motoren", page_motors),
    ("system", page_system),
]


# Rendert Statusseite index inkl. Seitenpunkten
def render_status(state, index):
    img = Image.new("1", (W, H), 0)
    d = ImageDraw.Draw(img)
    STATUS_PAGES[index][1](d, state)
    page_dots(d, index, len(STATUS_PAGES))
    return img


# ---------------------------------------------------------------- Sonderseiten

MENU_ITEMS = [
    "Software neu starten",
    "Rolle: STURM        >",
    "Spielernummer: 3    >",
    "IMU kalibrieren",
    "Display dimmen",
    "Herunterfahren",
    "Zurueck",
]


# Menue mit invertierter Auswahlzeile und Scrollbalken rechts
def render_menu(state, selected=1, items=MENU_ITEMS):
    img = Image.new("1", (W, H), 0)
    d = ImageDraw.Draw(img)
    header(d, "MENUE", state)
    rows = 5
    top = max(0, min(selected - 2, len(items) - rows))
    for i in range(rows):
        k = top + i
        if k >= len(items):
            break
        y = 12 + i * 10
        if k == selected:
            d.rectangle((0, y - 1, W - 5, y + 8), fill=1)
            NORMAL.draw(d, (2, y), items[k], 0)
        else:
            NORMAL.draw(d, (2, y), items[k], 1)
    # Scrollbalken
    track_y0, track_y1 = 11, 63
    d.line((W - 2, track_y0, W - 2, track_y1), fill=1)
    span = track_y1 - track_y0
    h = max(4, span * rows // len(items))
    y = track_y0 + (span - h) * top // max(1, len(items) - rows)
    d.rectangle((W - 3, y, W - 1, y + h), fill=1)
    return img


# Alarm: ganzer Bildschirm invertiert, damit er aus Entfernung auffaellt.
# Im Betrieb im Sekundentakt mit der Normalansicht abwechselnd blinken lassen.
def render_alarm(state, title="FEHLER", lines=("Motor 4  L-Knie", "keine Antwort, 3 s"), hint="MITTE = quittieren"):
    img = Image.new("1", (W, H), 1)
    d = ImageDraw.Draw(img)
    d.rectangle((2, 2, W - 3, H - 3), outline=0)
    text_center(d, 6, title, BIG, fill=0)
    for i, line in enumerate(lines[:2]):
        text_center(d, 25 + i * 10, line, NORMAL, fill=0)
    d.line((6, 47, W - 7, 47), fill=0)
    text_center(d, 51, hint, SMALL, fill=0)
    return img


# Startbild, solange die ROS-Nodes hochfahren
def render_boot(state, step=4, steps=7, what="walking"):
    img = Image.new("1", (W, H), 0)
    d = ImageDraw.Draw(img)
    text_center(d, 8, "BIT-BOTS", BIG)
    text_center(d, 26, "HAMBURG  ROBOCUP", SMALL)
    bar(d, 14, 38, 100, 8, step / steps)
    text_center(d, 51, f"starte {what}  {step}/{steps}", SMALL)
    return img


# Bild der Firmware selbst, wenn vom Roboterrechner nichts mehr kommt.
# Wird im STM32 gezeichnet (nicht in Python), steht hier nur als Vorlage.
def render_fallback(state, seconds=12):
    img = Image.new("1", (W, H), 0)
    d = ImageDraw.Draw(img)
    text_center(d, 4, "KEIN HOST", BIG)
    text_center(d, 22, f"seit {seconds} s keine Daten", NORMAL)
    d.line((0, 33, W - 1, 33), fill=1)
    SMALL.draw(d, (0, 39), "AKKU")
    NORMAL.draw(d, (26, 38), f"{state.battery_voltage:.1f} V")
    SMALL.draw(d, (0, 49), "FW")
    NORMAL.draw(d, (26, 48), "bitbots-oled 0.1")
    return img
