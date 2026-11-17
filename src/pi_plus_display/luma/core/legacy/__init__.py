# -*- coding: utf-8 -*-
# Copyright (c) 2017-18 Richard Hull and contributors
# See LICENSE.rst for details.

"""
These methods were originally present in the old MAX7219 driver, and are
preserved here only to aid migration away from that library. The functions are
denoted 'legacy' to discourage use - you are encouraged to use the various
drawing capabilities in the Pillow library instead.
"""

from luma.core.legacy.font import DEFAULT_FONT


def textsize(txt, font=None):
    """
    Calculates the bounding box of the text, as drawn in the specified font.
    This method is most useful for when the
    :py:class:`~luma.core.legacy.font.proportional` wrapper is used.

    :param txt: The text string to calculate the bounds for
    :type txt: str
    :param font: The font (from :py:mod:`luma.core.legacy.font`) to use.
    """
    font = font or DEFAULT_FONT
    src = [c for ascii_code in txt for c in font[ord(ascii_code)]]
    return (len(src), 8)


def text(draw, xy, txt, fill=None, font=None):
    """
    Draw a legacy font starting at :py:attr:`x`, :py:attr:`y` using the
    prescribed fill and font.

    :param draw: A valid canvas to draw the text onto.
    :type draw: PIL.ImageDraw
    :param txt: The text string to display (must be ASCII only).
    :type txt: str
    :param xy: An ``(x, y)`` tuple denoting the top-left corner to draw the
        text.
    :type xy: tuple
    :param fill: The fill color to use (standard Pillow color name or RGB
        tuple).
    :param font: The font (from :py:mod:`luma.core.legacy.font`) to use.
    """
    font = font or DEFAULT_FONT
    x, y = xy
    for ch in txt:
        for byte in font[ord(ch)]:
            for j in range(8):
                if byte & 0x01 > 0:
                    draw.point((x, y + j), fill=fill)

                byte >>= 1
            x += 1
