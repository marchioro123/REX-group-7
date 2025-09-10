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
        self.isTurning = False
        self.isDrivingForward = False

    def run(self):
        while True:
            try:
                if self.wait_until <= time.monotonic():
                    name, *args = self.cmd_queue.get(block=False)

                    if name == "turn":
                        self._turn(*args)
                    elif name == "turn_90_degrees":
                        self._turn_90_degrees(*args)
                    elif name == "drive_n_cm_forward":
                        self._drive_n_cm_forward(*args)

            except queue.Empty:
                if self.wait_until != 0:
                    with self.serial_lock:
                        self.wait_until = 0
                        self.isTurning = False
                        self.isDrivingForward = False
                        self.arlo.stop()

            time.sleep(0.01)

    def is_turning(self):
        return self.isTurning
    
    def is_driving_forward(self):
        return self.isDrivingForward
    
    def hard_stop(self):
        while True:
            try:
                self.cmd_queue.get(block=False)
            except queue.Empty:
                break

        with self.serial_lock:
            self.wait_until = 0
            self.isTurning = False
            self.isDrivingForward = False
            self.arlo.stop()

    def _turn(self, direction: int, seconds: float):
        LEFTSPEED, RIGHTSPEED = 57, 55
        ldir, rdir = (0, 1) if direction == 0 else (1, 0)

        with self.serial_lock:
            self.isTurning = True
            self.isDrivingForward = False
            self.wait_until = time.monotonic() + seconds
            self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, ldir, rdir)

    def _turn_90_degrees(self, direction: int):
        LEFTSPEED, RIGHTSPEED = 105, 100
        ldir, rdir = (0, 1) if direction == 0 else (1, 0)

        with self.serial_lock:
            self.isTurning = True
            self.isDrivingForward = False
            self.wait_until = time.monotonic() + 0.37
            self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, ldir, rdir)

    def _drive_n_cm_forward(self, speed: int, cm: float):
        if speed == 0:
            LEFTSPEED, RIGHTSPEED, k = 41, 40, 0.039
        elif speed == 1:
            LEFTSPEED, RIGHTSPEED, k = 86, 83, 0.0162
        else:
            LEFTSPEED, RIGHTSPEED, k = 127, 115, 0.0105

        duration = k * cm

        with self.serial_lock:
            self.isTurning = False
            self.isDrivingForward = True 
            self.wait_until = time.monotonic() + duration
            self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)