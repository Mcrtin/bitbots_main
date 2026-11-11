# Statusanzeige als Baustein fuer eine bestehende ROS-Node.
# Einbinden im Konstruktor der Node (z. B. in display.py):
#     from pi_plus_display.status_display import StatusDisplay
#     self.status_display = StatusDisplay(self)
# Die Nachrichten bekommt die Anzeige von den Callbacks der Node: self.status_display.source.on_*(msg).
# StatusDisplay rendert in einem Timer das 128x64-Bild und schickt nur geaenderte Framebuffer-Pages per UART an die Platine.
# Joystick-Ereignisse der Platine steuern Seiten und Menue.
# Parameter (alle mit Praefix "display."): siehe declare_parameter unten und config/display.yaml.

import os
import time

from rcl_interfaces.srv import GetParameters
from rclpy.clock import Clock, ClockType
from rclpy.parameter import parameter_value_to_python

from .app import DisplayApp
from .ros_source import RosStatusSource

SETTINGS_WARN_S = 10.0  # so lange darf das parameter_blackboard beim Start fehlen


class StatusDisplay:
    def __init__(self, node):
        self.node = node
        log = node.get_logger()

        self.source = RosStatusSource()
        self.app = DisplayApp(self.source)

        self.uart = None
        serial_port = self.node.serial_port
        baudrate = self.node.baudrate
        if serial_port and not os.path.exists(serial_port):
            log.warning(
                f"Statusdisplay: {serial_port} gibt es nicht, ohne Platine weiter"
            )
        elif serial_port:
            try:
                from .uart import StatusBoardUart

                self.uart = StatusBoardUart(serial_port, baudrate)
                log.info(f"Statusdisplay an {serial_port} ({baudrate} Baud)")
            except ModuleNotFoundError as e:
                log.error(
                    f"Statusdisplay: pyserial fehlt ({e}); in bitbots_main: "
                    "pixi add --feature ros 'pyserial>=3.5,<4'"
                )
            except Exception as e:
                log.error(
                    f"Statusdisplay: {serial_port} laesst sich nicht oeffnen, es wird nichts angezeigt: {e}"
                )

        # Steady Clock: die Anzeige soll auch laufen, wenn use_sim_time gesetzt ist und keine /clock kommt
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self._last_tick = time.monotonic()
        node.create_timer(1.0 / self.node.frame_rate, self._on_frame, clock=steady)
        node.create_timer(2.0, self.source.poll_local, clock=steady)
        node.create_timer(1.0, self.source.poll_hardware, clock=steady)

        self._settings_client = node.create_client(
            GetParameters, "parameter_blackboard/get_parameters"
        )
        self._settings_future = None
        self._settings_t0 = time.monotonic()
        self._settings_warned = False
        self._settings_timer = node.create_timer(
            1.0, self._poll_player_number, clock=steady
        )

    # bot_id vom parameter_blackboard, ohne zu warten: get_parameters_from_other_node
    # blockierte bis zu 20 s im Konstruktor; hier fragt ein 1-s-Timer, bis der Dienst da ist
    def _poll_player_number(self):
        if self._settings_future is not None:
            return
        if not self._settings_client.service_is_ready():
            waited = time.monotonic() - self._settings_t0
            if not self._settings_warned and waited > SETTINGS_WARN_S:
                self._settings_warned = True
                self.node.get_logger().warning(
                    "Statusdisplay: parameter_blackboard laeuft nicht, ohne Spielernummer weiter"
                )
            return
        request = GetParameters.Request(names=["bot_id"])
        self._settings_future = self._settings_client.call_async(request)
        self._settings_future.add_done_callback(self._on_player_number)

    def _on_player_number(self, future):
        self._settings_timer.cancel()
        try:
            bot_id = parameter_value_to_python(future.result().values[0])
        except Exception as e:
            self.node.get_logger().error(
                f"Statusdisplay: Antwort vom parameter_blackboard unbrauchbar: {e}"
            )
            return
        if bot_id is not None:
            self.source.set_player_number(bot_id)
        self.node.get_logger().info(f"Statusdisplay: Spieler {bot_id}")

    def _on_frame(self):
        now = time.monotonic()
        dt, self._last_tick = now - self._last_tick, now
        if self.uart:
            try:
                for key in self.uart.read_keys():
                    self.app.joystick(key)
            except OSError as e:
                self.node.get_logger().error(
                    f"Statusdisplay: Lesen vom UART fehlgeschlagen: {e}",
                    throttle_duration_sec=5.0,
                )
        self.app.tick(dt)
        if self.uart:
            try:
                self.uart.send_image(self.app.render())
            except OSError as e:
                self.uart.resend_all()
                self.node.get_logger().error(
                    f"Statusdisplay: Senden per UART fehlgeschlagen: {e}",
                    throttle_duration_sec=5.0,
                )

    def close(self):
        if self.uart:
            self.uart.close()
