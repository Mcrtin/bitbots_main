# Hardware-Seiten: Akku (livelybot_power), Rechner, Temperaturen und Netz-Interfaces (hardware_interfaces).
# Unbekannte Werte erscheinen als "--"; die Reihenfolge steht in pages.STATUS_PAGES.

from .state import TempSensor
from .widgets import BIG, NORMAL, SMALL, W, bar, header, text_center, text_right

TEMP_MIN_C, TEMP_MAX_C = 20.0, 100.0  # Bereich der Temperaturbalken
TEMP_HOT_C = 80.0  # ab hier wird die Zeile invertiert
BATTERY_STALE_S = 3.0
ROWS = 5


# Gestrichelte waagrechte Linie
def _dashed(draw, x0, x1, y, step=3):
    for x in range(x0, x1 + 1, step):
        draw.point((x, y), fill=1)


# Verlauf als Flaeche, neuester Wert rechts; lo..hi wird auf die Hoehe top..bottom abgebildet
def _area(draw, values, x0, x1, top, bottom, lo, hi):
    span = max(hi - lo, 1e-9)
    for i, v in enumerate(reversed(values)):
        x = x1 - i
        if x < x0:
            break
        y = bottom - round(max(0.0, min(1.0, (v - lo) / span)) * (bottom - top))
        draw.line((x, y, x, bottom), fill=1)


def _num(v, fmt="{:.0f}"):
    return "--" if v is None else fmt.format(v)


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


# Seite AKKU: Spannung, Strom, Leistung, Ladung, Spannungsverlauf, Temperatur und BMS-Fehler
def page_battery(draw, s):
    header(draw, "AKKU", s)
    v, a = s.battery_voltage, s.battery_current_a
    BIG.draw(draw, (0, 12), "--.-V" if v is None else f"{v:.1f}V")
    text_right(draw, W - 1, 12, _num(a, "{:.1f} A"))
    text_right(draw, W - 1, 21, "--" if v is None or a is None else f"{v * a:.0f} W")
    SMALL.draw(draw, (0, 31), "LADUNG")
    bar(draw, 26, 30, 78, 7, (s.battery_percent or 0) / 100)
    text_right(draw, W - 1, 30, f"{_num(s.battery_percent)}%")
    # Spannungsverlauf der letzten 2 min, Massstab aus Minimum und Maximum (mindestens 0,5 V)
    hist = s.battery_history
    top, bottom = 39, 50
    draw.line((0, bottom, W - 1, bottom), fill=1)
    if hist:
        lo, hi = min(hist), max(hist)
        if hi - lo < 0.5:
            lo, hi = (lo + hi) / 2 - 0.25, (lo + hi) / 2 + 0.25
        _area(draw, hist, 0, W - 1, top, bottom, lo, hi)
        SMALL.draw(draw, (0, 53), f"MIN {min(hist):.1f}V")
    if v is None:
        text_center(draw, 41, "KEIN AKKU-TOPIC", SMALL)
    elif s.battery_age_s is not None and s.battery_age_s > BATTERY_STALE_S:
        text_center(draw, 53, f"VOR {s.battery_age_s:.0f} S", SMALL)
    else:
        text_center(draw, 53, f"TEMP {_num(s.battery_temp_c)}C", SMALL)
    if s.bms_error:
        text = f"BMS {s.bms_error}"
        x = W - SMALL.width(text) - 2
        draw.rectangle((x - 2, 52, W - 1, 58), fill=1)
        SMALL.draw(draw, (x, 53), text, 0)
    else:
        text_right(draw, W - 1, 53, "BMS --" if s.bms_error is None else "BMS OK", SMALL)


# Seite RECHNER: CPU-Last mit Verlauf, Takt, Load, Laufzeit, RAM und Platte
def page_computer(draw, s):
    header(draw, "RECHNER", s)
    SMALL.draw(draw, (0, 13), "CPU")
    NORMAL.draw(draw, (16, 12), f"{_num(s.cpu_percent)}%")
    ghz = "--" if s.cpu_mhz is None else f"{s.cpu_mhz / 1000:.1f}"
    text_right(draw, W - 1, 13, f"{ghz} GHZ  {_num(s.cpu_temp_c)}C", SMALL)
    top, bottom = 21, 42
    _dashed(draw, 0, W - 1, top)  # 100 %
    draw.line((0, bottom, W - 1, bottom), fill=1)
    _area(draw, s.cpu_history, 0, W - 1, top, bottom, 0.0, 100.0)
    if s.load_avg:
        SMALL.draw(draw, (0, 45), "LOAD " + " ".join(f"{v:.2f}" for v in s.load_avg))
    else:
        SMALL.draw(draw, (0, 45), "LOAD --")
    text_right(draw, W - 1, 45, f"UP {_uptime(s.uptime_s)}", SMALL)
    if s.ram_used_gb is not None:
        SMALL.draw(draw, (0, 53), f"RAM {s.ram_used_gb:.1f}/{s.ram_total_gb:.1f}G {_num(s.ram_percent)}%")
    else:
        SMALL.draw(draw, (0, 53), f"RAM {_num(s.ram_percent)}%")
    text_right(draw, W - 1, 53, f"DISK {_num(s.disk_percent)}%", SMALL)


# Seite TEMPERATUR: alle Fuehler des Rechners und der Akku, heissester oben, ab TEMP_HOT_C invertiert
def page_temperatures(draw, s):
    header(draw, "TEMPERATUR", s)
    sensors = list(s.temps)
    if s.battery_temp_c is not None:
        sensors.append(TempSensor("AKKU", s.battery_temp_c))
    sensors.sort(key=lambda t: -t.celsius)
    if not sensors:
        text_center(draw, 28, "KEINE FUEHLER", SMALL)
        return
    for i, t in enumerate(sensors[:ROWS]):
        y = 12 + i * 8
        hot = t.celsius >= TEMP_HOT_C
        if hot:
            draw.rectangle((0, y - 1, W - 1, y + 5), fill=1)
        fill = 0 if hot else 1
        SMALL.draw(draw, (1, y), t.name, fill)
        frac = (t.celsius - TEMP_MIN_C) / (TEMP_MAX_C - TEMP_MIN_C)
        x0, width = 46, 56
        draw.rectangle((x0, y, x0 + width - 1, y + 4), outline=fill)
        inner = round((width - 2) * max(0.0, min(1.0, frac)))
        if inner > 0:
            draw.rectangle((x0 + 1, y + 1, x0 + inner, y + 3), fill=fill)
        text_right(draw, W - 2, y, f"{t.celsius:.0f}C", SMALL, fill)
    more = len(sensors) - ROWS
    footer = f"+{more} WEITERE, " if more > 0 else ""
    SMALL.draw(draw, (1, 54), f"{footer}AB {TEMP_HOT_C:.0f}C INVERTIERT")


# Seite NETZ-IO: je Interface Zustand, IPv4 und Bytes pro Sekunde empfangen/gesendet (can0 = Motorbus)
def page_net_io(draw, s):
    header(draw, "NETZ-IO", s)
    if not s.net_ifaces:
        text_center(draw, 28, "KEINE INTERFACES", SMALL)
        return
    for i, n in enumerate(s.net_ifaces[:ROWS]):
        y = 12 + i * 8
        SMALL.draw(draw, (1, y), n.name[:7])
        if not n.up:
            SMALL.draw(draw, (31, y), "AUS")
            continue
        SMALL.draw(draw, (31, y), n.ip or "-")
        text_right(draw, W - 1, y, f"{_rate(n.rx_bps)} {_rate(n.tx_bps)}", SMALL)
    SMALL.draw(draw, (1, 54), "IF  IPV4")
    text_right(draw, W - 1, 54, "EMPF SEND B/S", SMALL)
