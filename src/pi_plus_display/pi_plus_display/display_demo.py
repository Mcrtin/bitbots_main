# Demo-Modus: zeigt die Statusanzeige in einem Fenster am Rechner, die Pfeiltasten wirken wie der Joystick der Platine.
# Das Fenster sitzt anstelle der Platine am Ende des UART-Protokolls: es zeigt nur, was als Rahmen ankommt,
# und die Tasten gehen als Joystick-Rahmen zurueck.
# Nur zum Ausprobieren; zum Entfernen diese Datei loeschen und den Demo-Block in display.py.
# Einschalten: ros2 run pi_plus_display display --ros-args -p demo:=true
# Tasten: Pfeile = Joystick, Enter oder Leertaste = Mitte, Q oder Esc = Ende

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
SCALE = 6  # Fensterpixel pro Displaypixel
REFRESH_MS = 50


# Steht anstelle des seriellen Ports: nimmt die Rahmen entgegen und baut daraus das Bild nach,
# Tasten kommen als Joystick-Rahmen zurueck
class LoopbackSerial:
    def __init__(self) -> None:
        self._board = FramebufferReceiver()
        self._rx = bytearray()  # Joystick-Rahmen vom Fenster
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

    # Immer dasselbe Objekt, bis sich das Bild aendert (das Fenster vergleicht per Identitaet)
    def image(self) -> Image.Image:
        if self._dirty or self._image is None:
            self._dirty = False
            self._image = self._board.image()
        return self._image


# Oeffnet das Fenster und blockiert, bis es geschlossen wird; get_image liefert das aktuelle Bild,
# on_key bekommt "up", "down", "left", "right" oder "center"
def run_window(
    title: str,
    get_image: Callable[[], Image.Image | None],
    on_key: Callable[[Key], None],
) -> None:
    import tkinter as tk

    from PIL import ImageTk

    # Alle Tk-Objekte in ui, damit sie am Ende im selben Thread abgeraeumt werden koennen
    ui: dict[str, Any] = {"root": tk.Tk(), "img": None, "photo": None}
    root = ui["root"]
    root.title(title)
    root.configure(bg="black")
    root.resizable(False, False)
    ui["view"] = tk.Label(root, bg="black", bd=0, width=W * SCALE, height=H * SCALE)
    ui["view"].pack(padx=16, pady=(16, 8))
    hint = "Pfeile = Joystick    Enter/Leertaste = Mitte    Q = Ende"
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
            )  # Referenz halten, sonst verschwindet das Bild
            ui["view"].configure(image=ui["photo"], width=W * SCALE, height=H * SCALE)
        ui["root"].after(REFRESH_MS, refresh)

    ui["root"].bind("<Key>", key)
    refresh()
    ui["root"].mainloop()
    # Sonst bricht Tcl beim Beenden ab, wenn das Fenster im Nebenthread lief
    ui.clear()
    gc.collect()


# Prueft sofort, ob sich ein Fenster oeffnen laesst (tkinter vorhanden, Bildschirm erreichbar)
def check_window() -> None:
    try:
        import tkinter

        tkinter.Tk().destroy()
    except Exception as e:
        raise RuntimeError(f"Demo-Fenster laesst sich nicht oeffnen (tkinter, Bildschirm/DISPLAY?): {e}") from e


# Haengt das Fenster an einen LoopbackSerial; es laeuft in einem eigenen Thread, rclpy.spin bleibt im Hauptthread;
# Schliessen des Fensters beendet die Node wie Strg+C, ein Fehler im Fenster steht danach in node.fatal_error
def start_window(node: Any, serial: LoopbackSerial) -> None:
    def run() -> None:
        try:
            run_window(f"Statusdisplay {node.get_name()} (Demo)", serial.image, serial.press)
        except Exception as e:
            node.fatal_error = f"Demo-Fenster: {e}"
        _thread.interrupt_main()

    threading.Thread(target=run, name="display_demo", daemon=True).start()
