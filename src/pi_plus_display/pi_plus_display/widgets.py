# Wiederverwendbare Bausteine: Schriften, Kopfzeile, Akku, WLAN, Balken, Seitenpunkte

from luma.core.legacy import text as luma_text
from luma.core.legacy import textsize as luma_textsize
from luma.core.legacy.font import LCD_FONT, TINY_FONT, proportional, tolerant

_UMLAUTE = str.maketrans(
    {"ä": "ae", "ö": "oe", "ü": "ue", "Ä": "AE", "Ö": "OE", "Ü": "UE", "ß": "ss"}
)


class Font:
    # Bitmap-Font aus luma.core.legacy.font, gezeichnet mit luma.core.legacy.text.
    # proportional() schneidet leere Spalten ab und setzt eine Spalte Abstand
    # tolerant() ersetzt Zeichen, die der Font nicht kennt, durch "?"
    # y_offset: leere Bitzeilen oben in der Glyphe.

    def __init__(self, glyphs, y_offset=0, scale=1, upper=False):
        self.glyphs = tolerant(proportional(glyphs), missing="?")
        self.y_offset = y_offset
        self.scale = scale
        self.upper = upper  # Font hat keine für uns brauchbaren Kleinbuchstaben

    def _prep(self, text):
        text = text.translate(_UMLAUTE)
        return text.upper() if self.upper else text

    def width(self, text):
        w, _ = luma_textsize(self._prep(text), font=self.glyphs)
        return max(0, w - 1) * self.scale  # ohne Abstand nach dem letzten Zeichen

    def draw(self, draw, xy, text, fill=1):  # zeichnet text; xy -> links oben
        luma_text(
            _Scaled(draw, xy, self.scale),
            (0, -self.y_offset),
            self._prep(text),
            fill=fill,
            font=self.glyphs,
        )


class _Scaled:
    # nimmt punkte von luma_text und setzt sie skaliert ins Bild

    def __init__(self, draw, origin, scale):
        self.draw, self.origin, self.scale = draw, origin, scale

    def point(self, xy, fill):
        s = self.scale
        x, y = self.origin[0] + xy[0] * s, self.origin[1] + xy[1] * s
        if s == 1:
            self.draw.point((x, y), fill=fill)
        else:
            self.draw.rectangle((x, y, x + s - 1, y + s - 1), fill=fill)


# 3x5, ca. 32 Zeichen pro Zeile, nur Grossbuchstaben; Labels und Nebeninfos
SMALL = Font(TINY_FONT, y_offset=1, upper=True)
# 5x7, ca. 21 Zeichen pro Zeile; Standardschrift
NORMAL = Font(LCD_FONT)
# 5x7 doppelt, 14 px hoch, ca. 10 Zeichen; Spielzustand und Rolle
BIG = Font(LCD_FONT, scale=2)

W, H = 128, 64
HEADER_H = 9  # Zeile 0..7 Text, Zeile 9 Trennlinie
CONTENT_Y = 11
FOOTER_Y = 61


def text_center(draw, y, text, font=NORMAL, fill=1, x0=0, x1=W):
    x = x0 + (x1 - x0 - font.width(text)) // 2
    font.draw(draw, (x, y), text, fill)


def text_right(draw, x_right, y, text, font=NORMAL, fill=1):
    font.draw(draw, (x_right - font.width(text) + 1, y), text, fill)


# Akkusymbol 13x7 px, batterie in 4 Segmenten
def battery(draw, x, y, percent, fill=1):
    draw.rectangle((x, y, x + 11, y + 6), outline=fill)
    draw.rectangle((x + 12, y + 2, x + 12, y + 4), fill=fill)
    segs = 0 if percent is None or percent <= 5 else min(4, 1 + int(percent) // 25)
    for i in range(segs):
        sx = x + 2 + i * 2
        draw.rectangle((sx, y + 2, sx, y + 4), fill=fill)


# Kopfzeile: links Robotername + Seitentitel, rechts WLAN + Akku + Prozent.
# Bewusst nicht invertiert: grosse, dauerhaft leuchtende Flaechen brennen
# beim OLED schneller ein.
def header(draw, left, state, fill=1):

    NORMAL.draw(draw, (0, 0), state.wlan_ip, fill)
    NORMAL.draw(draw, (NORMAL.width(state.wlan_ip) + 2, 0), left, fill)
    xr = W - 1
    bat = None if state.battery_percent is None else min(state.battery_percent, 99)
    battery(draw, xr - 12, 0, bat, fill)
    text_right(draw, xr - 14, 0, "--" if bat is None else f"{bat:d}", NORMAL, fill)
    draw.line((0, HEADER_H, W - 1, HEADER_H), fill=fill)


# Seitenanzeige unten Mitte: aktiver Punkt als 3x3, andere als 1x1
def page_dots(draw, index, count, fill=1):
    spacing = 6
    x = (W - (count - 1) * spacing) // 2
    for i in range(count):
        cx = x + i * spacing
        if i == index:
            draw.rectangle((cx - 1, FOOTER_Y, cx + 1, FOOTER_Y + 2), fill=fill)
        else:
            draw.point((cx, FOOTER_Y + 1), fill=fill)


# Fortschrittsbalken, value 0..1
def bar(draw, x, y, w, h, value, fill=1):
    draw.rectangle((x, y, x + w - 1, y + h - 1), outline=fill)
    inner = int(round((w - 4) * max(0.0, min(1.0, value))))
    if inner > 0:
        draw.rectangle((x + 2, y + 2, x + 1 + inner, y + h - 3), fill=fill)


# Zeile 'Label   Wert' mit kleinem Label und Wert in Standardschrift
def label_value(draw, y, label, value, x_value=34, fill=1, font=NORMAL):
    SMALL.draw(draw, (0, y + 1), label, fill)
    font.draw(draw, (x_value, y), value, fill)


# Kleines Kaestchen 'OK' (Umriss) bzw. 'ERR' (invertiert)
def status_tag(draw, x, y, ok, text=None, fill=1):
    text = text or ("OK" if ok else "ERR")
    w = SMALL.width(text) + 4
    if ok:
        draw.rectangle((x, y, x + w - 1, y + 8), outline=fill)
        SMALL.draw(draw, (x + 2, y + 2), text, fill)
    else:
        draw.rectangle((x, y, x + w - 1, y + 8), fill=fill)
        SMALL.draw(draw, (x + 2, y + 2), text, 1 - fill)
    return w
