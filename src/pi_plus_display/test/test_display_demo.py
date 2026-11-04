# Tests fuer den Demo-Modus, fallen mit display_demo.py weg

from pi_plus_display.display_demo import KEYS, WindowBoard
from pi_plus_display.pages import render_status
from pi_plus_display.state import RobotState


class _Board:
    def __init__(self):
        self.sent, self.closed = [], False

    def send_image(self, img):
        self.sent.append(img)
        return 8

    def read_keys(self):
        return ["up"]

    def resend_all(self):
        pass

    def close(self):
        self.closed = True


def test_window_board_mirrors_images_and_merges_keys():
    board = _Board()
    win = WindowBoard(board)
    img = render_status(RobotState(), 0)
    assert win.send_image(img) == 8
    assert win.image is img and board.sent == [img]
    win.keys.put(KEYS["Right"])
    win.keys.put(KEYS["Return"])
    assert win.read_keys() == ["up", "right", "center"]
    assert win.read_keys() == ["up"]
    win.close()
    assert board.closed


def test_window_board_without_board():
    win = WindowBoard()
    assert win.send_image(render_status(RobotState(), 0)) == 0
    win.keys.put("left")
    assert win.read_keys() == ["left"]
