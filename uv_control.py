# uv_control.py
import RPi.GPIO as GPIO
from config import UV_RELAY_PIN, PIR_PIN
GPIO.setup(UV_RELAY_PIN, GPIO.OUT)
GPIO.setup(PIR_PIN, GPIO.IN)

def uv_on():
    # don't turn on if PIR detects human
    if GPIO.input(PIR_PIN) == GPIO.HIGH:
        print("PIR triggered: do not turn UV on")
        return
    GPIO.output(UV_RELAY_PIN, GPIO.HIGH)

def uv_off():
    GPIO.output(UV_RELAY_PIN, GPIO.LOW)
