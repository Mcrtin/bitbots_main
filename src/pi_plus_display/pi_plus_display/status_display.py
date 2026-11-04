# Statusanzeige als Baustein fuer eine bestehende ROS-Node.
# Einbinden im Konstruktor der Node (z. B. in display.py):
#     from pi_plus_display.status_display import StatusDisplay
#     self.status_display = StatusDisplay(self)
# Die Nachrichten bekommt die Anzeige von den Callbacks der Node: self.status_display.source.on_*(msg).
# StatusDisplay rendert in einem Timer das 128x64-Bild und schickt nur geaenderte Framebuffer-Pages per UART an die Platine.
# Joystick-Ereignisse der Platine steuern Seiten und Menue.
# Parameter (alle mit Praefix "display."): siehe declare_parameter unten und config/display.yaml.

import time

from rclpy.clock import Clock, ClockType

from .app import DisplayApp
from .ros_source import RosStatusSource


class StatusDisplay:
    def __init__(self, node):
        self.node = node
        log = node.get_logger()
        p = "display."

        def param(name, default):
            return node.declare_parameter(p + name, default).value

        serial_port = param("serial_port", "/dev/ttyS4")
        baudrate = param("baudrate", 115200)
        frame_rate = param("frame_rate", 10.0)
        self.png_out = param("png_out", "")  # zum Testen ohne Platine: Bild als PNG schreiben

        team_id, player_number = None, None
        if param("use_parameter_blackboard", True):
            team_id, player_number = self._read_game_settings()

        self.source = RosStatusSource(
            team_id=team_id,
            player_number=player_number,
            battery_empty_v=param("battery_empty_v", 21.0),
            battery_full_v=param("battery_full_v", 25.2),
        )
        self.app = DisplayApp(self.source)

        self.uart = None
        if serial_port:
            try:
                from .uart import StatusBoardUart

                self.uart = StatusBoardUart(serial_port, baudrate)
                log.info(f"Statusdisplay an {serial_port} ({baudrate} Baud)")
            except Exception as e:
                log.error(f"Statusdisplay: {serial_port} laesst sich nicht oeffnen, es wird nichts angezeigt: {e}")

        # Steady Clock: die Anzeige soll auch laufen, wenn use_sim_time gesetzt ist und keine /clock kommt
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._last_tick = time.monotonic()
        node.create_timer(1.0 / frame_rate, self._on_frame, clock=steady)
        node.create_timer(2.0, self.source.poll_local, clock=steady)

    # bot_id und team_id vom parameter_blackboard, wie bitbots_team_communication
    def _read_game_settings(self):
        from bitbots_utils.utils import get_parameters_from_other_node

        try:
            params = get_parameters_from_other_node(self.node, "parameter_blackboard", ["bot_id", "team_id"])
            return params["team_id"], params["bot_id"]
        except RuntimeError as e:
            self.node.get_logger().error(f"Statusdisplay: keine Spielernummer/Team-ID vom parameter_blackboard: {e}")
            return None, None

    def _on_frame(self):
        now = time.monotonic()
        dt, self._last_tick = now - self._last_tick, now
        if self.uart:
            try:
                for key in self.uart.read_keys():
                    self.app.joystick(key)
            except OSError as e:
                self.node.get_logger().error(
                    f"Statusdisplay: Lesen vom UART fehlgeschlagen: {e}", throttle_duration_sec=5.0
                )
        self.app.tick(dt)
        img = self.app.render()
        if self.uart:
            try:
                self.uart.send_image(img)
            except OSError as e:
                self.uart.resend_all()
                self.node.get_logger().error(
                    f"Statusdisplay: Senden per UART fehlgeschlagen: {e}", throttle_duration_sec=5.0
                )
        if self.png_out:
            img.save(self.png_out)

    def close(self):
        if self.uart:
            self.uart.close()
