# Reusable building blocks: fonts, header, battery, bars, page dots

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
    # Bitmap font from luma.core.legacy.font, drawn with luma.core.legacy.text.
    # proportional() trims empty columns and adds one column of spacing
    # tolerant() replaces characters the font does not know with "?"
    # y_offset: empty bit rows at the top of the glyph.

    def __init__(self, glyphs: Sequence[Sequence[int]], y_offset: int = 0, scale: int = 1, upper: bool = False) -> None:
        self.glyphs = tolerant(proportional(glyphs), missing="?")
        self.y_offset = y_offset
        self.scale = scale
        self.upper = upper  # font has no lowercase letters usable for us

    def _prep(self, text: str) -> str:
        text = text.translate(_UMLAUTE)
        return text.upper() if self.upper else text

    def width(self, text: str) -> int:
        w, _ = luma_textsize(self._prep(text), font=self.glyphs)
        return max(0, int(w) - 1) * self.scale  # without spacing after the last character

    def draw(self, draw: Draw, xy: XY, text: str, fill: int = 1) -> None:  # draws text; xy -> top left
        luma_text(
            _Scaled(draw, xy, self.scale),
            (0, -self.y_offset),
            self._prep(text),
            fill=fill,
            font=self.glyphs,
        )


class _Scaled:
    # takes points from luma_text and puts them scaled into the image

    def __init__(self, draw: Draw, origin: XY, scale: int) -> None:
        self.draw, self.origin, self.scale = draw, origin, scale

    def point(self, xy: XY, fill: int) -> None:
        s = self.scale
        x, y = self.origin[0] + xy[0] * s, self.origin[1] + xy[1] * s
        if s == 1:
            self.draw.point((x, y), fill=fill)
        else:
            self.draw.rectangle((x, y, x + s - 1, y + s - 1), fill=fill)


# 3x5, approx. 32 characters per line, uppercase only; labels and secondary info
SMALL = Font(TINY_FONT, y_offset=1, upper=True)
# 5x7, approx. 21 characters per line; default font
NORMAL = Font(LCD_FONT)
# 5x7 doubled, 14 px high, approx. 10 characters; game state and role
BIG = Font(LCD_FONT, scale=2)

W, H = 128, 64
HEADER_H = 9  # rows 0..7 text, row 9 separator line
FOOTER_Y = 61


def text_center(draw: Draw, y: int, text: str, font: Font = NORMAL, fill: int = 1, x0: int = 0, x1: int = W) -> None:
    x = x0 + (x1 - x0 - font.width(text)) // 2
    font.draw(draw, (x, y), text, fill)


def text_right(draw: Draw, x_right: int, y: int, text: str, font: Font = NORMAL, fill: int = 1) -> None:
    font.draw(draw, (x_right - font.width(text) + 1, y), text, fill)


# Upright battery icon, 5x8 px; the fill level has 5 steps
def battery_icon(draw: Draw, x: int, y: int, percent: int | None) -> None:
    draw.line((x + 1, y, x + 3, y), fill=1)  # terminal
    draw.rectangle((x, y + 1, x + 4, y + 7), outline=1)
    level = 0 if percent is None else round(min(percent, 100) * 5 / 100)
    if level:
        draw.rectangle((x + 1, y + 7 - level, x + 3, y + 6), fill=1)


# Header: IP address + page title on the left, percent + battery on the right.
# Deliberately not inverted: large, permanently lit areas burn in
# faster on an OLED.
def header(draw: Draw, title: str, state: RobotState) -> None:
    NORMAL.draw(draw, (0, 0), state.wlan_ip)
    NORMAL.draw(draw, (NORMAL.width(state.wlan_ip) + 4, 0), title)
    percent = state.battery.percent
    battery_icon(draw, W - 5, 0, percent)
    text_right(draw, W - 8, 0, "--" if percent is None else str(min(percent, 99)))  # two digits
    draw.line((0, HEADER_H, W - 1, HEADER_H), fill=1)


# Page indicator bottom centre: active dot as 3x3, others as 1x1
def page_dots(draw: Draw, index: int, count: int) -> None:
    spacing = 6
    x = (W - (count - 1) * spacing) // 2
    for i in range(count):
        cx = x + i * spacing
        if i == index:
            draw.rectangle((cx - 1, FOOTER_Y, cx + 1, FOOTER_Y + 2), fill=1)
        else:
            draw.point((cx, FOOTER_Y + 1), fill=1)


# Progress bar; percent 0..100, None = empty
def bar(draw: Draw, x: int, y: int, w: int, h: int, percent: float | None) -> None:
    draw.rectangle((x, y, x + w - 1, y + h - 1), outline=1)
    inner = round((w - 4) * max(0.0, min(100.0, percent or 0.0)) / 100)
    if inner > 0:
        draw.rectangle((x + 2, y + 2, x + 1 + inner, y + h - 3), fill=1)


# Row 'label   value' with a small label and the value in the default font
def label_value(draw: Draw, y: int, label: str, value: str, x_value: int = 26) -> None:
    SMALL.draw(draw, (0, y + 1), label)
    NORMAL.draw(draw, (x_value, y), value)
