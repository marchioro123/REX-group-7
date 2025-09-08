import threading
import queue
import time


class MotorThread(threading.Thread):
    def __init__(self, arlo, cmd_queue: queue.Queue):
        super().__init__(daemon=True)
        self.arlo = arlo
        self.cmd_queue = cmd_queue
        self.wait_until = 0

    def run(self):
        while True:
            try:
                if self.wait_until <= time.monotonic():
                    name, *args = self.cmd_queue.get(block=False)

                    if name == "turn_n_degrees_left":
                        self._turn_n_degrees_left(float(args[0]))
                    elif name == "turn_n_degrees_right":
                        self._turn_n_degrees_right(float(args[0]))
                    elif name == "drive_n_cm_forward":
                        self._drive_n_cm_forward(float(args[0]))

            except queue.Empty:
                if self.wait_until != 0:
                    self.arlo.stop()
                    self.wait_until = 0

            time.sleep(0.01)

    def hard_stop(self):
        while True:
            try:
                self.cmd_queue.get(block=False)
            except queue.Empty:
                break
        self.arlo.stop()
        self.wait_until = 0


    def _turn_n_degrees_left(self, n: float):
        LEFTSPEED, RIGHTSPEED = 50, 50
        self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 0, 1)

        duration = 2.71 * n / 360.0
        self.wait_until = time.monotonic() + duration

    def _turn_n_degrees_right(self, n: float):
        LEFTSPEED, RIGHTSPEED = 60, 50
        self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 0)

        duration = 3 * n / 360.0
        self.wait_until = time.monotonic() + duration

    def _drive_n_cm_forward(self, n: float):
        LEFTSPEED, RIGHTSPEED = 60, 50
        self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)

        duration = 3 * n
        self.wait_until = time.monotonic() + duration

    def _drive_circle_right(self):
        LEFTSPEED, RIGHTSPEED = 127, 40
        self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)

        duration = 4.3
        self.wait_until = time.monotonic() + duration

    def _drive_circle_left(self):
        LEFTSPEED, RIGHTSPEED = 44, 115
        self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)

        duration = 4.4
        self.wait_until = time.monotonic() + duration
