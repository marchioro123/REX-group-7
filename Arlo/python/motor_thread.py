import threading
import queue
import time

class MotorThread(threading.Thread):
    def __init__(self, arlo, cmd_queue: queue.Queue, serial_lock: threading.Lock):
        super().__init__(daemon=True)
        self.arlo = arlo
        self.cmd_queue = cmd_queue
        self.serial_lock = serial_lock

        self._wait_until = 0
        self._has_started = False
        self._is_turning = False
        self._is_drivingForward = False

    def run(self):
        while True:
            try:
                if self._wait_until <= time.monotonic():
                    name, *args = self.cmd_queue.get(block=False)

                    if name == "turn_n_degrees":
                        self._turn_n_degrees(*args)
                    elif name == "drive_n_cm_forward":
                        self._drive_n_cm_forward(*args)

            except queue.Empty:
                if self._wait_until != 0:
                    with self.serial_lock:
                        self._wait_until = 0
                        self._is_turning = False
                        self._is_drivingForward = False
                        self.arlo.stop()

            time.sleep(0.01)

    def has_started(self):
        with self.serial_lock:
            return self._has_started
        
    def is_turning(self):
        with self.serial_lock:
            return self._is_turning
    
    def is_driving_forward(self):
        with self.serial_lock:
            return self._is_drivingForward
        
    def clear_has_started(self):
        with self.serial_lock:
            self._has_started = False

    def hard_stop(self):
        while True:
            try:
                self.cmd_queue.get(block=False)
            except queue.Empty:
                break

        with self.serial_lock:
            self._wait_until = 0
            self._is_turning = False
            self._is_drivingForward = False
            self.arlo.stop()

    def _turn_n_degrees(self, degrees: float):
        if (degrees < 0):
            LEFTSPEED, RIGHTSPEED = 55, 55
            #duration = 0.0098 * abs(degrees) # calibration
            duration = 0.0117 * abs(degrees)
            with self.serial_lock:
                self._is_turning = True
                self._is_drivingForward = False
                self._wait_until = time.monotonic() + duration
                self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 0, 1)
                self._has_started = True

        else:
            LEFTSPEED, RIGHTSPEED = 55, 55
           # duration = 0.0097 * degrees # calibration
            duration = 0.0117 * abs(degrees)
            with self.serial_lock:
                self._is_turning = True
                self._is_drivingForward = False
                self._wait_until = time.monotonic() + duration
                self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 0)
                self._has_started = True

    def _drive_n_cm_forward(self, speed: int, cm: float):
        if speed == 0:
            #LEFTSPEED, RIGHTSPEED, k = 40, 40, 0.041 # calibration k (last value)
            LEFTSPEED, RIGHTSPEED, k = 40, 40, 0.055
        elif speed == 1:
            LEFTSPEED, RIGHTSPEED, k = 86, 86, 0.0162
        else:
            LEFTSPEED, RIGHTSPEED, k = 127, 115, 0.0105

        duration = k * cm

        with self.serial_lock:
            self._is_drivingForward = True 
            self._is_turning = False
            self._wait_until = time.monotonic() + duration
            self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, 1, 1)
            self._has_started = True






    # def _turn(self, direction: int, seconds: float):
    #     LEFTSPEED, RIGHTSPEED = 57, 55
    #     ldir, rdir = (0, 1) if direction == 0 else (1, 0)

    #     with self.serial_lock:
    #         self._is_turning = True
    #         self._is_drivingForward = False
    #         self._wait_until = time.monotonic() + seconds
    #         self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, ldir, rdir)

    # def _turn_90_degrees(self, direction: int):
    #     LEFTSPEED, RIGHTSPEED = 105, 100
    #     ldir, rdir = (0, 1) if direction == 0 else (1, 0)

    #     with self.serial_lock:
    #         self._is_turning = True
    #         self._is_drivingForward = False
    #         self._wait_until = time.monotonic() + 0.37
    #         self.arlo.go_diff(LEFTSPEED, RIGHTSPEED, ldir, rdir)