# Tests for the UART protocol (board.py); run without the board

import random

from pi_plus_display.board import (
    IMAGE_BYTES,
    MAGIC,
    REFRESH_S,
    FrameParser,
    FramebufferReceiver,
    StatusBoardUart,
    crc8,
    from_pages,
    image_frame,
    joystick_events,
    joystick_frame,
    to_pages,
)
from pi_plus_display.pages import render_status
from pi_plus_display.state import Motor, RobotState


def _image(page=0, **state):
    s = RobotState(**state)
    s.motors = [Motor(i + 1, f"m{i}", "left_leg", torque_nm=i * 1.5) for i in range(6)]
    return render_status(s, page)


# ------------------------------------------------------------- CRC and frames


def test_crc8_check_value_and_basics():
    assert crc8(b"123456789") == 0xF4  # CRC-8/SMBUS
    assert crc8(b"") == 0 and crc8(bytes(1024)) == 0
    assert crc8(b"\x03") == 0x09


def test_frame_layouts():
    key = joystick_frame("right")
    assert key == bytes([MAGIC, 3, crc8(b"\x03")]) and len(key) == 3
    data = bytes(range(256)) * 4
    frame = image_frame(data)
    assert len(frame) == 1 + 1024 + 1 and frame[0] == MAGIC and frame[1:-1] == data and frame[-1] == crc8(data)


def test_pages_bit_layout_and_roundtrip():
    img = _image().point(lambda _: 0)
    img.putpixel((5, 9), 1)  # page 1, bit 1
    data = to_pages(img)
    assert data[1 * 128 + 5] == 0b10 and sum(data) == 0b10
    full = _image(1)
    assert from_pages(to_pages(full)).tobytes() == full.convert("1").tobytes()


# ------------------------------------------------------------- Parser


def test_parser_split_garbage_and_release_free_stream():
    parser = FrameParser(1)
    stream = b"\x00\x13" + joystick_frame("right") + b"\xaa\x99" + joystick_frame("left") + joystick_frame("center")
    payloads = []
    for i in range(0, len(stream), 2):  # in small chunks like from the UART
        payloads += parser.feed(stream[i : i + 2])
    assert joystick_events(payloads) == ["right", "left", "center"]


def test_parser_rejects_bad_crc_and_resyncs():
    good = joystick_frame("up")
    bad = bytearray(joystick_frame("down"))
    bad[1] ^= 0x01  # one bit in the key code
    assert joystick_events(FrameParser(1).feed(bytes(bad) + good)) == ["up"]
    assert joystick_events(FrameParser(1).feed(bytes([MAGIC]) + good)) == ["up"]  # marker without a frame before it


def test_parser_ignores_unknown_key_codes():
    assert joystick_events(FrameParser(1).feed(bytes([MAGIC, 9, crc8(b"\x09")]))) == []


def test_pixel_data_containing_the_magic_is_fine():
    data = bytes([MAGIC]) * IMAGE_BYTES  # every data byte looks like a marker
    receiver = FramebufferReceiver()
    receiver.feed(image_frame(data))
    assert receiver.data == data


def test_every_single_bit_flip_in_an_image_frame_is_detected():
    data = bytes(random.Random(1).randrange(256) for _ in range(IMAGE_BYTES))
    frame = image_frame(data)
    positions = [*range(1, 40), *range(40, len(frame), 17), len(frame) - 1]  # marker excluded
    for i in positions:
        for bit in range(8):
            broken = bytearray(frame)
            broken[i] ^= 1 << bit
            assert FrameParser(IMAGE_BYTES).feed(bytes(broken)) == [], (i, bit)


def test_receiver_recovers_after_lost_byte():
    first = bytes(random.Random(2).randrange(256) for _ in range(IMAGE_BYTES))
    second = bytes(random.Random(3).randrange(256) for _ in range(IMAGE_BYTES))
    receiver = FramebufferReceiver()
    receiver.feed(image_frame(first)[:-5] + image_frame(first)[-4:])  # one byte lost
    assert receiver.data == bytes(IMAGE_BYTES)
    receiver.feed(image_frame(second))
    assert receiver.data == second


# ------------------------------------------------------------- StatusBoardUart


class _Serial:  # board that reads everything and sends keys back
    def __init__(self):
        self.board, self.rx, self.writes = FramebufferReceiver(), bytearray(), 0

    def write(self, data):
        self.writes += 1
        self.board.feed(data)

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, n):
        data, self.rx = bytes(self.rx[:n]), self.rx[n:]
        return data

    def close(self):
        pass


def test_uart_sends_changes_at_once_and_unchanged_images_once_per_refresh():
    t = [0.0]
    ser = _Serial()
    uart = StatusBoardUart(ser, clock=lambda: t[0])
    first, second = _image(0), _image(1)
    assert uart.send_image(first)
    assert ser.board.image().tobytes() == first.convert("1").tobytes()
    assert not uart.send_image(first)  # unchanged
    t[0] += REFRESH_S / 2
    assert not uart.send_image(first)
    t[0] += REFRESH_S
    assert uart.send_image(first)  # refresh
    assert uart.send_image(second)  # changed: immediately
    assert ser.board.image().tobytes() == second.convert("1").tobytes()
    assert ser.writes == 3


def test_failed_write_is_retried_with_the_next_image():
    class Broken(_Serial):
        def write(self, data):
            raise OSError("gone")

    uart = StatusBoardUart(Broken())
    img = _image()
    for _ in range(2):
        try:
            uart.send_image(img)
        except OSError:
            pass
    assert uart._last is None  # nothing recorded as sent


def test_uart_reads_joystick_frames():
    ser = _Serial()
    ser.rx += joystick_frame("right") + joystick_frame("down")
    uart = StatusBoardUart(ser)
    assert uart.read_keys() == ["right", "down"]
    assert uart.read_keys() == []
