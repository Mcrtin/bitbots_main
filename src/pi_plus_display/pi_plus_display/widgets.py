# Wiederverwendbare Bausteine: Schriften, Kopfzeile, Akku, Balken, Seitenpunkte

from collections.abc import Sequence

from luma.core.legacy import text as luma_text
from luma.core.legacy import textsize as luma_textsize
from luma.core.legacy.font import LCD_FONT, TINY_FONT, proportional, tolerant
from PIL import ImageDraw

from .state import RobotState

Draw = ImageDraw.ImageDraw
XY = tuple[int, int]

_UMLAUTE = str.maketrans(
    {"ä": "ae", "ö": "oe", "ü": "ue", "Ä": "AE", "Ö": "OE", "Ü": "UE", "ß": "ss"}
)


class Font:
    # Bitmap-Font aus luma.core.legacy.font, gezeichnet mit luma.core.legacy.text.
    # proportional() schneidet leere Spalten ab und setzt eine Spalte Abstand
    # tolerant() ersetzt Zeichen, die der Font nicht kennt, durch "?"
    # y_offset: leere Bitzeilen oben in der Glyphe.

    def __init__(self, glyphs: Sequence[Sequence[int]], y_offset: int = 0, scale: int = 1, upper: bool = False) -> None:
        self.glyphs = tolerant(proportional(glyphs), missing="?")
        self.y_offset = y_offset
        self.scale = scale
        self.upper = upper  # Font hat keine für uns brauchbaren Kleinbuchstaben

    def _prep(self, text: str) -> str:
        text = text.translate(_UMLAUTE)
        return text.upper() if self.upper else text

    def width(self, text: str) -> int:
        w, _ = luma_textsize(self._prep(text), font=self.glyphs)
        return max(0, int(w) - 1) * self.scale  # ohne Abstand nach dem letzten Zeichen

    def draw(self, draw: Draw, xy: XY, text: str, fill: int = 1) -> None:  # zeichnet text; xy -> links oben
        luma_text(
            _Scaled(draw, xy, self.scale),
            (0, -self.y_offset),
            self._prep(text),
            fill=fill,
            font=self.glyphs,
        )


class _Scaled:
    # nimmt punkte von luma_text und setzt sie skaliert ins Bild

    def __init__(self, draw: Draw, origin: XY, scale: int) -> None:
        self.draw, self.origin, self.scale = draw, origin, scale

    def point(self, xy: XY, fill: int) -> None:
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
FOOTER_Y = 61


def text_center(draw: Draw, y: int, text: str, font: Font = NORMAL, fill: int = 1, x0: int = 0, x1: int = W) -> None:
    x = x0 + (x1 - x0 - font.width(text)) // 2
    font.draw(draw, (x, y), text, fill)


def text_right(draw: Draw, x_right: int, y: int, text: str, font: Font = NORMAL, fill: int = 1) -> None:
    font.draw(draw, (x_right - font.width(text) + 1, y), text, fill)


# Akkusymbol hochkant, 5x8 px; der Fuellstand hat 5 Stufen
def battery_icon(draw: Draw, x: int, y: int, percent: int | None) -> None:
    draw.line((x + 1, y, x + 3, y), fill=1)  # Pol
    draw.rectangle((x, y + 1, x + 4, y + 7), outline=1)
    level = 0 if percent is None else round(min(percent, 100) * 5 / 100)
    if level:
        draw.rectangle((x + 1, y + 7 - level, x + 3, y + 6), fill=1)


# Kopfzeile: links IP-Adresse + Seitentitel, rechts Prozent + Akku.
# Bewusst nicht invertiert: grosse, dauerhaft leuchtende Flaechen brennen
# beim OLED schneller ein.
def header(draw: Draw, title: str, state: RobotState) -> None:
    NORMAL.draw(draw, (0, 0), state.wlan_ip)
    NORMAL.draw(draw, (NORMAL.width(state.wlan_ip) + 4, 0), title)
    percent = state.battery.percent
    battery_icon(draw, W - 5, 0, percent)
    text_right(draw, W - 8, 0, "--" if percent is None else str(min(percent, 99)))  # zweistellig
    draw.line((0, HEADER_H, W - 1, HEADER_H), fill=1)


# Seitenanzeige unten Mitte: aktiver Punkt als 3x3, andere als 1x1
def page_dots(draw: Draw, index: int, count: int) -> None:
    spacing = 6
    x = (W - (count - 1) * spacing) // 2
    for i in range(count):
        cx = x + i * spacing
        if i == index:
            draw.rectangle((cx - 1, FOOTER_Y, cx + 1, FOOTER_Y + 2), fill=1)
        else:
            draw.point((cx, FOOTER_Y + 1), fill=1)


# Fortschrittsbalken; percent 0..100, None = leer
def bar(draw: Draw, x: int, y: int, w: int, h: int, percent: float | None) -> None:
    draw.rectangle((x, y, x + w - 1, y + h - 1), outline=1)
    inner = round((w - 4) * max(0.0, min(100.0, percent or 0.0)) / 100)
    if inner > 0:
        draw.rectangle((x + 2, y + 2, x + 1 + inner, y + h - 3), fill=1)


# Zeile 'Label   Wert' mit kleinem Label und Wert in Standardschrift
def label_value(draw: Draw, y: int, label: str, value: str, x_value: int = 26) -> None:
    SMALL.draw(draw, (0, y + 1), label)
    NORMAL.draw(draw, (x_value, y), value)
