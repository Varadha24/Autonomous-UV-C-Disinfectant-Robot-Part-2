# navigator.py
import time, math
from motor_control import set_velocity, stop
from sensors import read_ultrasonic, pir_triggered
from uv_control import uv_on, uv_off
from config import US_SAFE_DIST

class Navigator:
    def __init__(self, localization, lidar_thread):
        self.loc = localization
        self.lidar = lidar_thread
        self.waypoints = []  # list of (x,y) in meters
        self.goal_tolerance = 0.08

    def set_waypoints(self, wps):
        self.waypoints = list(wps)

    def goto_waypoint(self, wx, wy):
        # simple proportional controller using localization estimate
        while True:
            x, y, th = self.loc.estimate()
            dx = wx - x
            dy = wy - y
            dist = math.hypot(dx, dy)
            if dist < self.goal_tolerance:
                stop()
                return True
            # obstacle check (ultrasonic)
            d_us = read_ultrasonic()
            if d_us is not None and d_us < US_SAFE_DIST:
                # stop & kill UV for safety
                stop()
                uv_off()
                print("Obstacle detected by US - waiting")
                time.sleep(1.0)
                continue
            # turn toward goal
            target_angle = math.atan2(dy, dx)
            angle_err = (target_angle - th + math.pi)%(2*math.pi) - math.pi
            # simple control
            linear_speed = 0.15 if abs(angle_err) < 0.35 else 0.0
            angular_speed = max(min(angle_err*1.2, 0.6), -0.6)
            set_velocity(linear_speed, angular_speed)
            # ensure UV is on while moving and PIR clear
            if not pir_triggered():
                uv_on()
            else:
                uv_off()
            time.sleep(0.05)

    def run_mission(self):
        for wp in self.waypoints:
            ok = self.goto_waypoint(wp[0], wp[1])
            if not ok:
                print("Failed to reach waypoint", wp)
        stop()
        uv_off()
