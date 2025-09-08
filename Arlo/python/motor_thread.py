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
                        self._drive_n_cm_forward(*args)

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


    def _turn_n_degrees_left(self, degree: float):
        LEFTSPEED, RIGHTSPEED = 105, 100
        self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 0, 1)

        duration = 1.71 * (degree - 5) / 360.0
        self.wait_until = time.monotonic() + duration


    def _turn_n_degrees_right(self, n: float):
        LEFTSPEED, RIGHTSPEED = 105, 100
        self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 0)

        duration = 1.65 * n / 360.0
        self.wait_until = time.monotonic() + duration

    def _drive_n_cm_forward(self, speed: int, cm: float):
        if speed == 0:
            LEFTSPEED, RIGHTSPEED = 41, 40
            self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)

            duration = 0.039 * cm
            self.wait_until = time.monotonic() + duration
        elif speed == 1:
            LEFTSPEED, RIGHTSPEED = 86, 83
            self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)

            duration = 0.0162 * cm
            self.wait_until = time.monotonic() + duration
        else:
            LEFTSPEED, RIGHTSPEED = 127, 115
            self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)

            duration = 0.0105 * cm
            self.wait_until = time.monotonic() + duration
    
    # def _drive_circle_right(self):
    #     LEFTSPEED, RIGHTSPEED = 127, 40
    #     self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)

    #     duration = 4.3
    #     self.wait_until = time.monotonic() + duration

    # def _drive_circle_left(self):
    #     LEFTSPEED, RIGHTSPEED = 44, 115
    #     self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)

    #     duration = 4.4
    #     self.wait_until = time.monotonic() + duration
