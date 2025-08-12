# motor_control.py
import RPi.GPIO as GPIO
import time
from config import *

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup motor pins and pwm
GPIO.setup(LEFT_MOTOR_IN1, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_IN2, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
GPIO.setup(LEFT_MOTOR_PWM, GPIO.OUT)
GPIO.setup(RIGHT_MOTOR_PWM, GPIO.OUT)

left_pwm = GPIO.PWM(LEFT_MOTOR_PWM, 1000)
right_pwm = GPIO.PWM(RIGHT_MOTOR_PWM, 1000)
left_pwm.start(0)
right_pwm.start(0)

def _set_motor(in1, in2, pwm, speed):
    if speed > 0:
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(min(abs(speed)*100, 100))
    elif speed < 0:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)
        pwm.ChangeDutyCycle(min(abs(speed)*100, 100))
    else:
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.LOW)
        pwm.ChangeDutyCycle(0)

def set_velocity(v_linear, v_angular):
    """
    Simple differential drive converter.
    v_linear in m/s, v_angular in rad/s
    """
    # compute wheel speeds (m/s)
    v_left = v_linear - (v_angular * WHEEL_BASE / 2.0)
    v_right = v_linear + (v_angular * WHEEL_BASE / 2.0)

    # map to -1..1 duty fraction using MAX_SPEED
    left_cmd = max(min(v_left / MAX_SPEED, 1.0), -1.0)
    right_cmd = max(min(v_right / MAX_SPEED, 1.0), -1.0)

    _set_motor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, left_pwm, left_cmd)
    _set_motor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, right_pwm, right_cmd)

def stop():
    _set_motor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, left_pwm, 0)
    _set_motor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, right_pwm, 0)

def cleanup():
    stop()
    left_pwm.stop()
    right_pwm.stop()
    GPIO.cleanup()
