# Tests fuer das Demo-Fenster ohne Fenster: LoopbackSerial als Gegenstelle des UART-Protokolls

from pi_plus_display.board import StatusBoardUart, to_pages
from pi_plus_display.display_demo import LoopbackSerial
from pi_plus_display.pages import render_status
from pi_plus_display.state import RobotState


def test_window_image_comes_through_the_protocol():
    ser = LoopbackSerial()
    uart = StatusBoardUart(ser)
    img = render_status(RobotState(), 0)
    uart.send_image(img)
    shown = ser.image()
    assert to_pages(shown) == to_pages(img)
    assert ser.image() is shown  # unveraendert: dasselbe Objekt
    uart.send_image(render_status(RobotState(), 1))
    assert ser.image() is not shown


def test_window_keys_come_back_as_joystick_frames():
    ser = LoopbackSerial()
    uart = StatusBoardUart(ser)
    ser.press("right")
    ser.press("center")
    assert uart.read_keys() == ["right", "center"]
    assert uart.read_keys() == []


def test_loopback_has_no_real_port_and_ignores_close():
    ser = LoopbackSerial()
    assert ser.in_waiting == 0 and ser.read(10) == b""
    ser.close()
