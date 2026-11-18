# Connection to the status board (STM32 with custom firmware, SSD1309 display) over UART, 8N1.
# Each direction has exactly one message of fixed length:
#   robot -> board: AA, 1024-byte image in SSD1309 layout, CRC-8
#   board -> robot: AA, key code, CRC-8
# AA is the marker for resynchronising after an error; the CRC-8 (over everything after the marker) checks the content.
# A frame is only accepted if the CRC matches; otherwise only the marker is dropped and the search continues.
# The image is sent on change and otherwise once per second (REFRESH_S), so the board heals itself after
# disturbances. There are no acknowledgements. Key codes: JOYSTICK_KEYS; only key presses are sent.
# The frames are a proposal and must be agreed with the firmware.

import time
from collections.abc import Callable
from typing import Literal, Protocol

from PIL import Image

MAGIC = 0xAA

WIDTH, HEIGHT = 128, 64
PAGES = HEIGHT // 8  # SSD1309 in page mode: 8 pages of 8 rows
IMAGE_BYTES = PAGES * WIDTH  # one byte per column and page: 8 vertical pixels, bit 0 at the top
REFRESH_S = 1.0  # an unchanged image is also sent this often, so the board is correct again after disturbances

Key = Literal["up", "down", "left", "right", "center"]
JOYSTICK_KEYS: dict[int, Key] = {0: "up", 1: "down", 2: "left", 3: "right", 4: "center"}
KEY_CODES = {name: code for code, name in JOYSTICK_KEYS.items()}


# PIL image 128x64 -> 1024 bytes in SSD1309 layout
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


# 1024 bytes in SSD1309 layout -> PIL image 128x64 (counterpart of to_pages)
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


# CRC-8 with polynomial 0x07, initial value 0, no reflection and no final XOR (CRC-8/SMBUS)
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
    # Finds frames AA, payload_len bytes, CRC-8 in a byte stream. Incomplete frames stay in the buffer;
    # on a wrong CRC only the marker is dropped and the search continues from the next byte.

    def __init__(self, payload_len: int) -> None:
        self._payload_len = payload_len
        self._buf = bytearray()

    # Returns the payloads of all complete, valid frames
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


# Counterpart of the board: rebuilds the display contents from the image frames
class FramebufferReceiver:
    def __init__(self) -> None:
        self._parser = FrameParser(IMAGE_BYTES)
        self.data = bytes(IMAGE_BYTES)

    def feed(self, raw: bytes) -> None:
        for payload in self._parser.feed(raw):
            self.data = payload

    def image(self) -> Image.Image:
        return from_pages(self.data)


# key codes -> key names
def joystick_events(payloads: list[bytes]) -> list[Key]:
    return [JOYSTICK_KEYS[p[0]] for p in payloads if p[0] in JOYSTICK_KEYS]


# What StatusBoardUart needs from the serial port (pyserial.Serial satisfies it)
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
        self._last: bytes | None = None  # last image sent
        self._sent_at = 0.0
        self._keys = FrameParser(1)

    # Sends the image if it has changed or the last send was REFRESH_S ago; True if sent.
    # If the write fails, the old state is kept, so the next image tries again.
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
