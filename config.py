# config.py
MAP_PGM = "map.pgm"        # occupancy grid of prototype ward (black=occupied)
MAP_YAML = "map.yaml"     # optional: resolution & origin if you have it

# Raspberry Pi GPIO pins (change to match wiring)
LEFT_MOTOR_PWM = 18
LEFT_MOTOR_IN1 = 23
LEFT_MOTOR_IN2 = 24

RIGHT_MOTOR_PWM = 19
RIGHT_MOTOR_IN1 = 27
RIGHT_MOTOR_IN2 = 22

# encoder pins
LEFT_ENCODER_PIN = 5
RIGHT_ENCODER_PIN = 6

# ultrasonic (HC-SR04) pins (front)
US_TRIG = 17
US_ECHO = 4

# PIR sensor pin
PIR_PIN = 21

# UV-C relay pin
UV_RELAY_PIN = 20

# emergency stop (wired to GPIO input)
ESTOP_PIN = 12

# LiDAR serial port (RPLIDAR typical)
RPLIDAR_PORT = "/dev/ttyUSB0"

# robot geometry
WHEEL_DIAMETER = 0.065   # meters
WHEEL_BASE = 0.18        # meters center to center

# control params
MAX_SPEED = 0.3          # m/s
TURN_SPEED = 0.4         # rad/s

# particle filter params
NUM_PARTICLES = 400

# ultrasonic safety distance (m)
US_SAFE_DIST = 0.4

# PIR safety: if PIR triggered, UV must be off
