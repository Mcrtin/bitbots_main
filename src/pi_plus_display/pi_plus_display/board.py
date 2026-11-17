# Verbindung zur Statusplatine (STM32 mit eigener Firmware, SSD1309-Display) per UART, 8N1.
# Beide Richtungen haben genau eine Nachricht fester Laenge:
#   Roboter -> Platine: AA, 1024 Byte Bild im SSD1309-Layout, CRC-8
#   Platine -> Roboter: AA, Tastencode, CRC-8
# AA ist die Marke zum Wiederfinden nach einem Fehler; die CRC-8 (ueber alles nach der Marke) prueft den Inhalt.
# Ein Rahmen wird erst angenommen, wenn die CRC stimmt, sonst wird nur die Marke verworfen und weitergesucht.
# Das Bild wird bei Aenderung und sonst einmal pro Sekunde gesendet (REFRESH_S), damit die Platine sich nach
# Stoerungen selbst heilt. Es gibt keine Quittungen. Tastencodes: JOYSTICK_KEYS, gesendet wird nur das Druecken.
# Die Rahmen sind ein Vorschlag und muessen mit der Firmware abgestimmt werden.

import time
from collections.abc import Callable
from typing import Literal, Protocol

from PIL import Image

MAGIC = 0xAA

WIDTH, HEIGHT = 128, 64
PAGES = HEIGHT // 8  # SSD1309 im Page-Modus: 8 Pages a 8 Zeilen
IMAGE_BYTES = PAGES * WIDTH  # ein Byte je Spalte und Page: 8 senkrechte Pixel, Bit 0 oben
REFRESH_S = 1.0  # auch ein unveraendertes Bild wird so oft gesendet, damit die Platine nach Stoerungen wieder stimmt

Key = Literal["up", "down", "left", "right", "center"]
JOYSTICK_KEYS: dict[int, Key] = {0: "up", 1: "down", 2: "left", 3: "right", 4: "center"}
KEY_CODES = {name: code for code, name in JOYSTICK_KEYS.items()}


# PIL-Bild 128x64 -> 1024 Bytes im SSD1309-Layout
def to_pages(img: Image.Image) -> bytes:
    img = img.convert("1")
    px = img.load()
    assert px is not None
    out = bytearray(IMAGE_BYTES)
    for page in range(PAGES):
        for x in range(WIDTH):
            b = 0
            for bit in range(8):
                if px[x, page * 8 + bit]:
                    b |= 1 << bit
            out[page * WIDTH + x] = b
    return bytes(out)


# 1024 Bytes im SSD1309-Layout -> PIL-Bild 128x64 (Gegenstueck zu to_pages)
def from_pages(data: bytes | bytearray) -> Image.Image:
    img = Image.new("1", (WIDTH, HEIGHT))
    px = img.load()
    assert px is not None
    for page in range(PAGES):
        for x in range(WIDTH):
            b = data[page * WIDTH + x]
            for bit in range(8):
                px[x, page * 8 + bit] = (b >> bit) & 1
    return img


def _crc8_step(crc: int) -> int:
    for _ in range(8):
        crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


_CRC8_TABLE = [_crc8_step(i) for i in range(256)]


# CRC-8 mit Polynom 0x07, Startwert 0, ohne Spiegelung und ohne Abschluss-XOR (CRC-8/SMBUS)
def crc8(data: bytes | bytearray) -> int:
    crc = 0
    for byte in data:
        crc = _CRC8_TABLE[crc ^ byte]
    return crc


def _frame(payload: bytes) -> bytes:
    return bytes([MAGIC]) + payload + bytes([crc8(payload)])


def image_frame(data: bytes) -> bytes:
    assert len(data) == IMAGE_BYTES
    return _frame(data)


def joystick_frame(key: Key) -> bytes:
    return _frame(bytes([KEY_CODES[key]]))


class FrameParser:
    # Sucht Rahmen AA, payload_len Bytes, CRC-8 in einem Bytestrom. Unvollstaendige Rahmen bleiben im Puffer,
    # bei falscher CRC wird nur die Marke verworfen und ab dem naechsten Byte weitergesucht.

    def __init__(self, payload_len: int) -> None:
        self._payload_len = payload_len
        self._buf = bytearray()

    # Gibt die Payloads aller vollstaendigen, gueltigen Rahmen zurueck
    def feed(self, data: bytes) -> list[bytes]:
        self._buf += data
        size = self._payload_len + 2
        frames: list[bytes] = []
        while True:
            start = self._buf.find(MAGIC)
            if start < 0:
                self._buf.clear()
                return frames
            del self._buf[:start]
            if len(self._buf) < size:
                return frames
            payload = bytes(self._buf[1 : size - 1])
            if crc8(payload) == self._buf[size - 1]:
                frames.append(payload)
                del self._buf[:size]
            else:
                del self._buf[:1]


# Gegenstueck zur Platine: baut aus den Bild-Rahmen das Abbild des Displays nach
class FramebufferReceiver:
    def __init__(self) -> None:
        self._parser = FrameParser(IMAGE_BYTES)
        self.data = bytes(IMAGE_BYTES)

    def feed(self, raw: bytes) -> None:
        for payload in self._parser.feed(raw):
            self.data = payload

    def image(self) -> Image.Image:
        return from_pages(self.data)


# Tastencodes -> Tastennamen
def joystick_events(payloads: list[bytes]) -> list[Key]:
    return [JOYSTICK_KEYS[p[0]] for p in payloads if p[0] in JOYSTICK_KEYS]


# Das, was StatusBoardUart vom seriellen Port braucht (pyserial.Serial erfuellt es)
class SerialPort(Protocol):
    @property
    def in_waiting(self) -> int: ...

    def write(self, data: bytes, /) -> int | None: ...

    def read(self, size: int, /) -> bytes: ...

    def close(self) -> None: ...


class StatusBoardUart:
    def __init__(self, ser: SerialPort, clock: Callable[[], float] = time.monotonic) -> None:
        self.ser = ser
        self._clock = clock
        self._last: bytes | None = None  # zuletzt gesendetes Abbild
        self._sent_at = 0.0
        self._keys = FrameParser(1)

    # Schickt das Bild, wenn es sich geaendert hat oder das letzte Senden REFRESH_S her ist; True, wenn gesendet.
    # Schlaegt das Schreiben fehl, bleibt der alte Stand, das naechste Bild versucht es also erneut.
    def send_image(self, img: Image.Image) -> bool:
        data = to_pages(img)
        now = self._clock()
        if data == self._last and now - self._sent_at < REFRESH_S:
            return False
        self.ser.write(image_frame(data))
        self._last, self._sent_at = data, now
        return True

    def read_keys(self) -> list[Key]:
        waiting = self.ser.in_waiting
        if not waiting:
            return []
        return joystick_events(self._keys.feed(self.ser.read(waiting)))

    def close(self) -> None:
        self.ser.close()
