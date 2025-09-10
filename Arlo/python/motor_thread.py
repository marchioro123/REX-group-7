import threading
import queue
import time

class MotorThread(threading.Thread):
    def __init__(self, arlo, cmd_queue: queue.Queue, serial_lock: threading.Lock):
        super().__init__(daemon=True)
        self.arlo = arlo
        self.cmd_queue = cmd_queue
        self.serial_lock = serial_lock

        self.wait_until = 0
        self.uninterruptible = False

    def run(self):
        while True:
            try:
                if self.wait_until <= time.monotonic():
                    name, *args = self.cmd_queue.get(block=False)

                    if name == "turn_90_degrees":
                        self._turn_90_degrees(*args)
                    elif name == "drive_n_cm_forward":
                        self._drive_n_cm_forward(*args)

            except queue.Empty:
                if self.wait_until != 0:
                    with self.serial_lock:
                        self.arlo.stop()
                    self.wait_until = 0
                    self.uninterruptible = False


            time.sleep(0.01)

    def is_busy(self) -> bool:
        return self.uninterruptible
    
    def hard_stop(self):
        if self.uninterruptible:
            return
        
        while True:
            try:
                self.cmd_queue.get(block=False)
            except queue.Empty:
                break
        self.wait_until = 0
        with self.serial_lock:
            self.arlo.stop()

    def _turn_90_degrees(self, direction: int, uninterruptible = False):
        if direction == 0:
            LEFTSPEED, RIGHTSPEED = 105, 100
            with self.serial_lock:
                self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 0, 1)
            duration = 0.37
        else:
            LEFTSPEED, RIGHTSPEED = 105, 100
            with self.serial_lock:
                self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 0)
            duration = 0.37

        self.uninterruptible = uninterruptible
        self.wait_until = time.monotonic() + duration

    def _drive_n_cm_forward(self, speed: int, cm: float, uninterruptible = False):
        if speed == 0:
            LEFTSPEED, RIGHTSPEED = 41, 40
            with self.serial_lock:
                self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)
            duration = 0.039 * cm
        elif speed == 1:
            LEFTSPEED, RIGHTSPEED = 86, 83
            with self.serial_lock:
                self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)
            duration = 0.0162 * cm
        else:
            LEFTSPEED, RIGHTSPEED = 127, 115
            with self.serial_lock:
                self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)
            duration = 0.0105 * cm

        self.uninterruptible = uninterruptible
        self.wait_until = time.monotonic() + duration