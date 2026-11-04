# Bedienlogik der Anzeige, unabhaengig von Datenquelle und Ausgabe.
# DisplayApp bekommt Joystick-Ereignisse ("left", "right", "up", "down",
# "center", "back"), liest die Daten aus einer DataSource und liefert mit
# render() das naechste 128x64-Bild. Ob das Bild in einem Fenster landet oder
# per UART auf der Platine, entscheidet der Aufrufer.

import time

from PIL import ImageDraw

from .pages import STATUS_PAGES, render_alarm, render_boot, render_fallback, render_menu, render_status
from .state import ROLES
from .widgets import NORMAL, W, text_center

BOOT_SECONDS = 3.5
LOW_BATTERY_PERCENT = 10


class DisplayApp:
    def __init__(self, source):
        self.source = source
        self.page = 0
        self.mode = "boot"  # boot, status, menu, fallback
        self.boot_t0 = time.monotonic()
        self.menu_sel = 0
        self.alarm = None  # (titel, zeile1, zeile2) solange nicht quittiert
        self.toast = None  # (text, ablaufzeit)
        self.dim = False
        self._prev = None  # letzter Zustand, um neue Fehler zu erkennen

    # ------------------------------------------------------------- Joystick

    def joystick(self, key):
        if self.mode == "boot":
            return
        if self.mode == "fallback":
            self.mode, self.boot_t0 = "boot", time.monotonic()
            return
        if self.alarm and key == "center":
            self.alarm = None
            self.show_toast("quittiert")
            return
        if self.mode == "status":
            n = len(STATUS_PAGES)
            if key == "left":
                self.page = (self.page - 1) % n
            elif key == "right":
                self.page = (self.page + 1) % n
            elif key == "center":
                self.mode, self.menu_sel = "menu", 0
        elif self.mode == "menu":
            items = self.menu_items(self.source.snapshot())
            if key == "up":
                self.menu_sel = (self.menu_sel - 1) % len(items)
            elif key == "down":
                self.menu_sel = (self.menu_sel + 1) % len(items)
            elif key in ("left", "back"):
                self.mode = "status"
            elif key in ("center", "right"):
                self.menu_action(self.menu_sel)

    def menu_items(self, s):
        return [
            "Software neu starten",
            f"Rolle: {s.role} >",
            f"Spielernummer: {s.player_number} >",
            "IMU kalibrieren",
            "Display hell" if self.dim else "Display dimmen",
            "Herunterfahren",
            "Zurueck",
        ]

    def menu_action(self, i):
        s = self.source.snapshot()
        if i == 0:
            self.mode, self.boot_t0 = "boot", time.monotonic()
        elif i == 1:
            self.source.set_role(ROLES[(ROLES.index(s.role) + 1) % len(ROLES)] if s.role in ROLES else ROLES[0])
        elif i == 2:
            self.source.set_player_number(s.player_number % 6 + 1)
        elif i == 3:
            self.show_toast("IMU kalibriert")
        elif i == 4:
            self.dim = not self.dim
        elif i == 5:
            self.mode = "fallback"
        elif i == 6:
            self.mode = "status"

    def show_toast(self, text, seconds=1.5):
        self.toast = (text, time.monotonic() + seconds)

    # ------------------------------------------------------------- Ablauf

    def tick(self, dt):
        self.source.update(dt)
        s = self.source.snapshot()
        self._check_alarms(s)
        self._prev = s
        if self.mode == "boot" and time.monotonic() - self.boot_t0 > BOOT_SECONDS:
            self.mode = "status"

    # Alarm nur beim Uebergang in den Fehlerzustand, damit Quittieren haelt
    def _check_alarms(self, s):
        p = self._prev
        if p is None:
            return
        ok_before = {m.id for m in p.motors if m.ok}
        for m in s.motors:
            if not m.ok and m.id in ok_before:
                self.alarm = ("FEHLER", f"Motor {m.id}  {m.name}", "keine Antwort")
        ok_nodes = {n.name for n in p.nodes if n.ok}
        for n in s.nodes:
            if not n.ok and n.name in ok_nodes:
                self.alarm = ("NODE", f"{n.name} ausgefallen", "neu starten: Menue")
        if s.battery_percent < LOW_BATTERY_PERCENT <= p.battery_percent:
            self.alarm = ("AKKU", f"{s.battery_voltage:.1f} V", "bitte laden")

    # ------------------------------------------------------------- Rendern

    def render(self):
        shown = _display_values(self.source.snapshot())
        if self.mode == "boot":
            steps = ["vision", "walking", "behavior", "locali.", "gc_cli", "teamcom", "oled"]
            k = min(len(steps), int((time.monotonic() - self.boot_t0) / BOOT_SECONDS * len(steps)) + 1)
            return render_boot(shown, k, len(steps), steps[k - 1])
        if self.mode == "fallback":
            return render_fallback(shown, int(time.monotonic() - self.boot_t0))
        if self.alarm and int(time.monotonic() * 2) % 2 == 0:  # blinkt mit 1 Hz
            title, l1, l2 = self.alarm
            return render_alarm(shown, title, (l1, l2), "MITTE = QUITTIEREN")
        if self.mode == "menu":
            img = render_menu(shown, self.menu_sel, self.menu_items(shown))
        else:
            img = render_status(shown, self.page)
        if self.toast and time.monotonic() < self.toast[1]:
            d = ImageDraw.Draw(img)
            d.rectangle((10, 22, W - 11, 40), fill=0, outline=1)
            text_center(d, 28, self.toast[0], NORMAL)
        return img


# Ganzzahlige Anzeigewerte; Quellen duerfen mit float rechnen
def _display_values(s):
    s.battery_percent = int(round(s.battery_percent))
    s.seconds_remaining = int(s.seconds_remaining)
    s.penalty_seconds = int(s.penalty_seconds)
    return s
