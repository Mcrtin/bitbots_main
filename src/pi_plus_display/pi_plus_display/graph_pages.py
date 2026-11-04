# Seiten mit Graphen fuers Debugging: Feldkarte, Last je Gelenk, Beinlast-Verlauf, Datenfluss.
# Entstanden aus den echten Node-Daten (node_output.txt); die Reihenfolge steht in pages.STATUS_PAGES.

import math

from .widgets import SMALL, W, bar, header, text_right

# Spielfeld HSL KidSize in Metern (Laenge x Breite), Mittelkreis-Radius
FIELD_L, FIELD_W, CIRCLE_R = 9.0, 6.0, 0.75
# Grenze fuer Drehmoment-Ueberlast in livelybot_serial (diagnostics.torque_overload_threshold)
OVERLOAD_NM = 10.0
LOAD_SCALE_NM = 15.0


# Gestrichelte waagrechte Linie
def _dashed(draw, x0, x1, y, fill=1, step=3):
    for x in range(x0, x1 + 1, step):
        draw.point((x, y), fill=fill)


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
    draw.line((x0 + fw + 2, cy - goal, x0 + fw + 2, cy + goal), fill=1)  # gegnerisches Tor rechts

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
    draw.line((px, py, px + round(6 * math.cos(a)), py - round(6 * math.sin(a))), fill=1)
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
    SMALL.draw(draw, (0, bottom - round(OVERLOAD_NM / LOAD_SCALE_NM * height) - 2), f"{OVERLOAD_NM:.0f}")
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
