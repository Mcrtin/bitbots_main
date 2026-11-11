# Bedienlogik der Anzeige, unabhaengig von Datenquelle und Ausgabe.
# DisplayApp bekommt Joystick-Ereignisse ("left", "right", "up", "down",
# "center", "back"), liest die Daten aus einer DataSource und liefert mit
# render() das naechste 128x64-Bild. Ob das Bild in einem Fenster landet oder
# per UART auf der Platine, entscheidet der Aufrufer.

import time

from PIL import ImageDraw

from .pages import (
    STATUS_PAGES,
    render_alarm,
    render_status,
)
from .state import ROLES
from .widgets import NORMAL, W, text_center

LOW_BATTERY_PERCENT = 10


class DisplayApp:
    def __init__(self, source):
        self.source = source
        self.page = 0
        self.alarm = None  # (titel, zeile1, zeile2, end_time)
        self._prev = None  # letzter Zustand, um neue Fehler zu erkennen

    def joystick(self, key):
        if self.alarm:
            self.alarm = None
        n = len(STATUS_PAGES)
        if key == "left":
            self.page = (self.page - 1) % n
        elif key == "right":
            self.page = (self.page + 1) % n

    def tick(self, dt):
        self.source.update(dt)
        s = self.source.snapshot()
        self._check_alarms(s)
        self._prev = s

    def _check_alarms(self, s):
        p = self._prev
        if p is None:
            return
        ok_before = {m.id for m in p.motors if m.ok}
        for m in s.motors:
            if not m.ok and m.id in ok_before:
                self.alarm = (
                    "FEHLER",
                    f"Motor {m.id}  {m.name}",
                    "keine Antwort",
                    time.monotonic() + 30,
                )
        ok_nodes = {n.name for n in p.nodes if n.ok}
        for n in s.nodes:
            if not n.ok and n.name in ok_nodes:
                self.alarm = (
                    "NODE",
                    f"{n.name} ausgefallen",
                    "neu starten: Menue",
                    time.monotonic() + 30,
                )
        if (
            s.battery_percent is not None
            and p.battery_percent is not None
            and s.battery_percent < LOW_BATTERY_PERCENT <= p.battery_percent
        ):
            self.alarm = (
                "AKKU",
                f"{s.battery_voltage:.1f} V",
                "bitte laden",
                time.monotonic() + 60 * 5,
            )
        if s.bms_error and not p.bms_error:
            self.alarm = (
                "AKKU",
                f"BMS-Fehler {s.bms_error}",
                "Akku pruefen",
                time.monotonic() + 30,
            )

    # ------------------------------------------------------------- Rendern

    def render(self):
        shown = self.source.snapshot()
        now = time.monotonic()
        if self.alarm and int(now * 2) % 2 == 0:  # blinkt mit 1 Hz
            title, l1, l2, end = self.alarm
            if end < now:
                self.alarm = None
                return
            return render_alarm(shown, title, (l1, l2), "")
        img = render_status(shown, self.page)
        return img
