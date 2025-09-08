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
            if self.wait_until and time.monotonic() >= self.wait_until:
                self.arlo.stop()
                self.wait_until = 0

            if self.wait_until == 0:
                try:
                    name, *args = self.cmd_queue.get(block=False)
                    if name == "turn_90_degrees":
                        self._turn_90_degrees(*args)
                    elif name == "drive_n_cm_forward":
                        self._drive_n_cm_forward(*args)
                except queue.Empty:
                    pass

            time.sleep(0.01)

    def hard_stop(self):
        while True:
            try:
                self.cmd_queue.get(block=False)
            except queue.Empty:
                break
        self.wait_until = 0
        self.arlo.stop()

    def _turn_90_degrees(self, direction: int):
        if direction == 0:
            LEFTSPEED, RIGHTSPEED = 105, 100
            self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 0, 1)
            duration = 0.37
        else:
            LEFTSPEED, RIGHTSPEED = 105, 100
            self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 0)
            duration = 0.37

        self.wait_until = time.monotonic() + duration

    def _drive_n_cm_forward(self, speed: int, cm: float):
        if speed == 0:
            LEFTSPEED, RIGHTSPEED = 41, 40
            self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)
            duration = 0.039 * cm
        elif speed == 1:
            LEFTSPEED, RIGHTSPEED = 86, 83
            self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)
            duration = 0.0162 * cm
        else:
            LEFTSPEED, RIGHTSPEED = 127, 115
            self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)
            duration = 0.0105 * cm

        self.wait_until = time.monotonic() + duration