# UART-Verbindung zur Statusplatine (STM32 mit eigener Firmware).
# Richtung Platine: fertige Bilder als Framebuffer-Pages (Typ 0x24, siehe framebuffer.py).
# Richtung Roboter: Joystick-Ereignisse (Typ 0x30), Payload [Taste, gedrueckt].
# Die Tastencodes sind ein Vorschlag und muessen mit der Firmware abgestimmt werden.

from .framebuffer import FRAME_HEAD, FRAME_TAIL, framebuffer_frames

TYPE_JOYSTICK = 0x30
JOYSTICK_KEYS = {0: "up", 1: "down", 2: "left", 3: "right", 4: "center"}


class FrameParser:
    # Sucht Rahmen A5 5A LEN TYP DATEN 66 47 74 in einem Bytestrom.
    # LEN zaehlt TYP und DATEN. Unvollstaendige Rahmen bleiben im Puffer, kaputte werden verworfen.

    def __init__(self):
        self._buf = bytearray()

    # Gibt die vollstaendigen Rahmen als Liste von (typ, payload) zurueck
    def feed(self, data):
        self._buf += data
        frames = []
        while True:
            start = self._buf.find(FRAME_HEAD)
            if start < 0:
                # Ein einzelnes A5 am Ende kann der Anfang des naechsten Kopfes sein
                del self._buf[: max(0, len(self._buf) - 1)]
                return frames
            del self._buf[:start]
            if len(self._buf) < 3:
                return frames
            length = self._buf[2]
            end = 3 + length + len(FRAME_TAIL)
            if len(self._buf) < end:
                return frames
            if length < 1 or self._buf[3 + length : end] != FRAME_TAIL:
                del self._buf[:1]  # kein gueltiger Rahmen, ab dem naechsten Byte weitersuchen
                continue
            frames.append((self._buf[3], bytes(self._buf[4 : 3 + length])))
            del self._buf[:end]


# Rahmen -> Tastennamen, die DisplayApp.joystick() versteht. Nur Druecken zaehlt
def joystick_events(frames):
    keys = []
    for typ, payload in frames:
        if typ == TYPE_JOYSTICK and len(payload) >= 2 and payload[1]:
            key = JOYSTICK_KEYS.get(payload[0])
            if key:
                keys.append(key)
    return keys


class StatusBoardUart:
    def __init__(self, port, baudrate):
        #TODO: no module named serial
        import serial  # pyserial, erst hier importiert, damit Tests ohne Platine laufen

        self.ser = serial.Serial(port, baudrate, timeout=0, write_timeout=0.1)
        self._last = None  # zuletzt gesendetes 1024-Byte-Abbild
        self._parser = FrameParser()

    # Schickt nur die Pages, die sich seit dem letzten Bild geaendert haben
    def send_image(self, img):
        frames, self._last = framebuffer_frames(img, changed_only_from=self._last)
        for f in frames:
            self.ser.write(f)
        return len(frames)

    # Beim naechsten Bild alle Pages schicken, z. B. nach einem Neustart der Platine
    def resend_all(self):
        self._last = None

    def read_keys(self):
        waiting = self.ser.in_waiting
        if not waiting:
            return []
        return joystick_events(self._parser.feed(self.ser.read(waiting)))

    def close(self):
        self.ser.close()
