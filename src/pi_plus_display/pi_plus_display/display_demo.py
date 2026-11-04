# Demo-Modus: zeigt die Statusanzeige in einem Fenster am Rechner, die Pfeiltasten wirken wie der Joystick der Platine.
# Nur zum Ausprobieren; zum Entfernen diese Datei loeschen und den Demo-Block in display.py.
# Live mit ROS, Daten aus display.py: ros2 run pi_plus_display display (Parameter demo in display.py, derzeit fest auf true)
# Ohne ROS, Log aus display.py im Kreis: python3 -m pi_plus_display.display_demo node_output.txt
# Tasten: Pfeile = Joystick, Enter oder Leertaste = Mitte, Q oder Esc = Ende

import _thread
import argparse
import gc
import queue
import threading
import time

from .widgets import H, W

KEYS = {
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


class WindowBoard:
    # Steht in StatusDisplay an der Stelle der UART-Platine: merkt sich das letzte Bild fuers Fenster und liefert die Tasten;
    # ist eine echte Platine offen, bekommt sie die Bilder weiterhin

    def __init__(self, board=None):
        self.board = board
        self.image = None
        self.keys = queue.Queue()

    def send_image(self, img):
        self.image = img
        return self.board.send_image(img) if self.board else 0

    # Tasten aus dem Fenster und, falls vorhanden, vom Joystick der Platine
    def read_keys(self):
        keys = self.board.read_keys() if self.board else []
        while not self.keys.empty():
            keys.append(self.keys.get_nowait())
        return keys

    def resend_all(self):
        if self.board:
            self.board.resend_all()

    def close(self):
        if self.board:
            self.board.close()


# Oeffnet das Fenster und blockiert, bis es geschlossen wird; get_image liefert das aktuelle Bild,
# on_key bekommt "up", "down", "left", "right" oder "center", on_tick laeuft vor jedem Neuzeichnen
def run_window(title, get_image, on_key, on_tick=None):
    import tkinter as tk

    from PIL import Image, ImageTk

    # Alle Tk-Objekte in ui, damit sie am Ende im selben Thread abgeraeumt werden koennen
    ui = {"root": tk.Tk(), "img": None, "photo": None}
    root = ui["root"]
    root.title(title)
    root.configure(bg="black")
    root.resizable(False, False)
    ui["view"] = tk.Label(root, bg="black", bd=0, width=W * SCALE, height=H * SCALE)
    ui["view"].pack(padx=16, pady=(16, 8))
    hint = "Pfeile = Joystick    Enter/Leertaste = Mitte    Q = Ende"
    tk.Label(root, text=hint, fg="#8a8a8a", bg="black").pack(pady=(0, 12))
    del root

    def key(event):
        if event.keysym in ("q", "Q", "Escape"):
            ui["root"].destroy()
        elif event.keysym in KEYS:
            on_key(KEYS[event.keysym])

    def refresh():
        if on_tick:
            on_tick()
        img = get_image()
        if img is not None and img is not ui["img"]:
            big = img.convert("L").resize((W * SCALE, H * SCALE), Image.NEAREST)
            ui["img"], ui["photo"] = img, ImageTk.PhotoImage(big)  # Referenz halten, sonst verschwindet das Bild
            ui["view"].configure(image=ui["photo"], width=W * SCALE, height=H * SCALE)
        ui["root"].after(REFRESH_MS, refresh)

    ui["root"].bind("<Key>", key)
    refresh()
    ui["root"].mainloop()
    # Sonst bricht Tcl beim Beenden ab, wenn das Fenster im Nebenthread lief
    ui.clear()
    gc.collect()


# Haengt das Fenster an die StatusDisplay einer laufenden Node; das Fenster laeuft in einem eigenen Thread,
# rclpy.spin bleibt im Hauptthread; Schliessen des Fensters beendet die Node wie Strg+C
def start_window(node, status_display):
    board = WindowBoard(status_display.uart)
    status_display.uart = board

    def run():
        try:
            run_window(f"Statusdisplay {node.get_name()} (Demo)", lambda: board.image, board.keys.put)
        except Exception as e:  # kein Bildschirm (DISPLAY fehlt) oder kein tkinter: ohne Fenster weiterlaufen
            status_display.uart = board.board
            node.get_logger().error(f"Demo-Fenster laesst sich nicht oeffnen, es laeuft ohne: {e}")
            return
        _thread.interrupt_main()

    threading.Thread(target=run, name="display_demo", daemon=True).start()


# Ohne ROS: spielt das Log in Echtzeit im Kreis ab und zeigt die Seiten im Fenster
def main():
    from .app import DisplayApp
    from .log_replay import LogReplay
    from .ros_source import RosStatusSource

    ap = argparse.ArgumentParser(description="Statusanzeige im Fenster, Daten aus einem Log von display.py")
    ap.add_argument("log", help="Log mit msg=...-Zeilen, z. B. node_output.txt")
    args = ap.parse_args()

    events = LogReplay(args.log).events
    start = events[0][0]
    span = events[-1][0] - start + 0.1  # Abstand zwischen zwei Durchlaeufen
    t0 = time.monotonic()
    pos = {"i": 0, "round": 0, "last": t0, "poll": t0 - 1.0}
    source = RosStatusSource(clock=lambda: time.monotonic() - t0)
    source._state.robot_name = "LOG"
    app = DisplayApp(source)

    def tick():
        now = time.monotonic()
        while (events[pos["i"]][0] - start) + pos["round"] * span <= now - t0:
            _, handler, msg = events[pos["i"]]
            getattr(source, handler)(msg)
            pos["i"] += 1
            if pos["i"] == len(events):
                pos["i"], pos["round"] = 0, pos["round"] + 1
        if now - pos["poll"] >= 1.0:  # wie die Timer in StatusDisplay: Rechner-Werte dieses Rechners
            source.poll_hardware()
            source.poll_local()
            pos["poll"] = now
        app.tick(now - pos["last"])
        pos["last"] = now

    run_window(f"Statusdisplay (Demo, {args.log})", app.render, app.joystick, on_tick=tick)


if __name__ == "__main__":
    main()
