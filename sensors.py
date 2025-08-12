# sensors.py
import threading, time, math
from rplidar import RPLidar
import RPi.GPIO as GPIO
from config import *
import smbus2

# --- LiDAR thread ---
class LidarThread(threading.Thread):
    def __init__(self, port=RPLIDAR_PORT):
        super().__init__()
        self.port = port
        self.lidar = None
        self.scan = []   # latest scan as list of (angle_deg, dist_mm)
        self.running = False

    def run(self):
        try:
            self.lidar = RPLidar(self.port)
            self.lidar.start_motor()
            self.running = True
            for meas in self.lidar.iter_scans(max_buf_meas=500):
                # meas: [(quality, angle, distance), ...]
                parsed = [(m[1], m[2]) for m in meas if m[2] > 0]
                self.scan = parsed
                if not self.running:
                    break
        except Exception as e:
            print("LiDAR error:", e)

    def stop(self):
        self.running = False
        if self.lidar:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()

    def get_scan(self):
        return list(self.scan)

# --- Ultrasonic HC-SR04 simple function ---
GPIO.setup(US_TRIG, GPIO.OUT)
GPIO.setup(US_ECHO, GPIO.IN)
def read_ultrasonic():
    # returns distance in meters (None on fail)
    GPIO.output(US_TRIG, False)
    time.sleep(0.01)
    GPIO.output(US_TRIG, True)
    time.sleep(0.00001)
    GPIO.output(US_TRIG, False)

    start = time.time()
    timeout = start + 0.02
    while GPIO.input(US_ECHO) == 0 and time.time() < timeout:
        start = time.time()
    stop = time.time()
    timeout2 = stop + 0.02
    while GPIO.input(US_ECHO) == 1 and time.time() < timeout2:
        stop = time.time()
    dt = stop - start
    if dt <= 0:
        return None
    dist = (dt * 343.0) / 2.0
    return dist

# --- PIR sensor ---
GPIO.setup(PIR_PIN, GPIO.IN)
def pir_triggered():
    return GPIO.input(PIR_PIN) == GPIO.HIGH

# --- Emergency Stop read ---
GPIO.setup(ESTOP_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
def estop_pressed():
    return GPIO.input(ESTOP_PIN) == GPIO.LOW   # active low

# --- IMU (MPU6050) very simple heading read ---
class IMU:
    def __init__(self, bus=1, addr=0x68):
        self.bus = smbus2.SMBus(bus)
        self.addr = addr
        try:
            # wake up
            self.bus.write_byte_data(self.addr, 0x6B, 0)
        except Exception as e:
            print("IMU init fail:", e)

    def read_accel_gyro(self):
        # simplified: return yaw estimate placeholder (needs AHRS/filter in real)
        return 0.0
