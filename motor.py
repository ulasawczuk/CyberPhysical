import time
import board
import digitalio
from adafruit_motor import motor
import pwmio

pwm1 = pwmio.PWMOut(board.D21) #A1
pwm2 = pwmio.PWMOut(board.D16) #A2
pwm3 = pwmio.PWMOut(board.D25) #B1
pwm4 = pwmio.PWMOut(board.D24) #B2

motor1 = motor.DCMotor(pwm1, pwm2)
motor2 = motor.DCMotor(pwm3, pwm4)

motor1.decay_mode = (
    motor.SLOW_DECAY
)

motor2.decay_mode = (
    motor.SLOW_DECAY
)

print("Forwards slow")
motor1.throttle = 0.5
motor2.throttle = 0
time.sleep(1)

print("Forwards")
motor1.throttle = 1
motor2.throttle = 1
time.sleep(1)

print("Backwards slow")
motor1.throttle = -0.5
motor2.throttle = -0.5
time.sleep(1)

print("Backwards")
motor1.throttle = -1
motor2.throttle = -1
time.sleep(1)

print("Stop")
motor1.throttle = 0
motor2.throttle = 0
time.sleep(1)

print("Spin freely")
motor1.throttle = None
motor2.throttle = None
time.sleep(1)
