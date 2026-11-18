# Demo mode: shows the status display in a window on the computer; the arrow keys act like the board's joystick.
# The window sits in place of the board at the end of the UART protocol: it only shows what arrives as frames,
# and the keys go back as joystick frames.
# Only for trying things out; to remove it, delete this file and the demo block in display.py.
# Enable: ros2 run pi_plus_display display --ros-args -p demo:=true
# Keys: arrows = joystick, Enter or space = centre, Q or Esc = quit

import _thread
import gc
import threading
from collections.abc import Callable
from typing import Any

from PIL import Image

from .board import FramebufferReceiver, Key, joystick_frame
from .widgets import H, W

KEYS: dict[str, Key] = {
    "Up": "up",
    "Down": "down",
    "Left": "left",
    "Right": "right",
    "Return": "center",
    "KP_Enter": "center",
    "space": "center",
}
SCALE = 6  # window pixels per display pixel
REFRESH_MS = 50


# Stands in for the serial port: receives the frames and rebuilds the image from them;
# keys come back as joystick frames
class LoopbackSerial:
    def __init__(self) -> None:
        self._board = FramebufferReceiver()
        self._rx = bytearray()  # joystick frames from the window
        self._image: Image.Image | None = None
        self._dirty = True

    def write(self, data: bytes, /) -> int:
        self._board.feed(data)
        self._dirty = True
        return len(data)

    @property
    def in_waiting(self) -> int:
        return len(self._rx)

    def read(self, n: int, /) -> bytes:
        data = bytes(self._rx[:n])
        del self._rx[:n]
        return data

    def close(self) -> None:
        pass

    def press(self, key: Key) -> None:
        self._rx += joystick_frame(key)

    # Always the same object until the image changes (the window compares by identity)
    def image(self) -> Image.Image:
        if self._dirty or self._image is None:
            self._dirty = False
            self._image = self._board.image()
        return self._image


# Opens the window and blocks until it is closed; get_image returns the current image,
# on_key receives "up", "down", "left", "right" or "center"
def run_window(
    title: str,
    get_image: Callable[[], Image.Image | None],
    on_key: Callable[[Key], None],
) -> None:
    import tkinter as tk

    from PIL import ImageTk

    # All Tk objects in ui so they can be cleaned up in the same thread at the end
    ui: dict[str, Any] = {"root": tk.Tk(), "img": None, "photo": None}
    root = ui["root"]
    root.title(title)
    root.configure(bg="black")
    root.resizable(False, False)
    ui["view"] = tk.Label(root, bg="black", bd=0, width=W * SCALE, height=H * SCALE)
    ui["view"].pack(padx=16, pady=(16, 8))
    hint = "Arrows = joystick    Enter/Space = centre    Q = quit"
    tk.Label(root, text=hint, fg="#8a8a8a", bg="black").pack(pady=(0, 12))
    del root

    def key(event: Any) -> None:
        if event.keysym in ("q", "Q", "Escape"):
            ui["root"].destroy()
        elif event.keysym in KEYS:
            on_key(KEYS[event.keysym])

    def refresh() -> None:
        img = get_image()
        if img is not None and img is not ui["img"]:
            big = img.convert("L").resize((W * SCALE, H * SCALE), Image.Resampling.NEAREST)
            ui["img"], ui["photo"] = img, ImageTk.PhotoImage(
                big
            )  # keep a reference, otherwise the image disappears
            ui["view"].configure(image=ui["photo"], width=W * SCALE, height=H * SCALE)
        ui["root"].after(REFRESH_MS, refresh)

    ui["root"].bind("<Key>", key)
    refresh()
    ui["root"].mainloop()
    # Otherwise Tcl aborts on exit if the window ran in a side thread
    ui.clear()
    gc.collect()


# Checks right away whether a window can be opened (tkinter available, display reachable)
def check_window() -> None:
    try:
        import tkinter

        tkinter.Tk().destroy()
    except Exception as e:
        raise RuntimeError(f"Demo window cannot be opened (tkinter, screen/DISPLAY?): {e}") from e


# Attaches the window to a LoopbackSerial; it runs in its own thread, rclpy.spin stays in the main thread;
# closing the window stops the node like Ctrl+C, an error in the window ends up in node.fatal_error
def start_window(node: Any, serial: LoopbackSerial) -> None:
    def run() -> None:
        try:
            run_window(f"Status display {node.get_name()} (demo)", serial.image, serial.press)
        except Exception as e:
            node.fatal_error = f"Demo window: {e}"
        _thread.interrupt_main()

    threading.Thread(target=run, name="display_demo", daemon=True).start()
