# sudo apt-get update
# sudo apt-get install python3-rpi.gpio


import RPi.GPIO as GPIO
import time

# GPIO setup
GPIO.setmode(GPIO.BCM)

# Define GPIO pins for L298N
ENA = 18
IN1 = 24
IN2 = 25
ENB = 23
IN3 = 17
IN4 = 27

# Setup GPIO pins
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Enable pins as PWM
pwmA = GPIO.PWM(ENA, 100)
pwmB = GPIO.PWM(ENB, 100)

# Start PWM with 0 duty cycle (off)
pwmA.start(0)
pwmB.start(0)

def motor_control(pwm, in1, in2, speed, direction):
    pwm.ChangeDutyCycle(speed)
    if direction == 'forward':
        GPIO.output(in1, GPIO.HIGH)
        GPIO.output(in2, GPIO.LOW)
    elif direction == 'backward':
        GPIO.output(in1, GPIO.LOW)
        GPIO.output(in2, GPIO.HIGH)

try:
    while True:
        # Move both pairs of motors forward
        motor_control(pwmA, IN1, IN2, 75, 'forward')
        motor_control(pwmB, IN3, IN4, 75, 'forward')
        time.sleep(5)

        # Stop all motors
        pwmA.ChangeDutyCycle(0)
        pwmB.ChangeDutyCycle(0)
        time.sleep(2)

        # Move both pairs of motors backward
        motor_control(pwmA, IN1, IN2, 75, 'backward')
        motor_control(pwmB, IN3, IN4, 75, 'backward')
        time.sleep(5)

        # Stop all motors
        pwmA.ChangeDutyCycle(0)
        pwmB.ChangeDutyCycle(0)
        time.sleep(2)

except KeyboardInterrupt:
    pass

# Cleanup
pwmA.stop()
pwmB.stop()
GPIO.cleanup()
