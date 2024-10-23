import board
import pwmio
from gpiozero import RotaryEncoder
from adafruit_motor import motor
from simple_pid import PID

class MotorController:

    def __init__(self, pwm1_pin, pwm2_pin, enc_a_pin, enc_b_pin, target_rpm=40):
        # Motor setup
        pwm1 = pwmio.PWMOut(pwm1_pin)
        pwm2 = pwmio.PWMOut(pwm2_pin)
        self.motor = motor.DCMotor(pwm1, pwm2)
        self.motor.decay_mode = motor.SLOW_DECAY

        if pwm1_pin == board.D21:
            self.side = "L" 
        else:
            self.side = "R"

        # Encoder setup
        self.encoder = RotaryEncoder(enc_a_pin, enc_b_pin, max_steps=0)

        # PID constants
        self.K_P = 0.002
        self.K_I = 0.0001
        self.K_D = 0.0001
        self.pid = PID(self.K_P, self.K_I, self.K_D, setpoint=target_rpm)
        self.pid.output_limits = (-1, 1) 

        self.target_rpm = target_rpm
        self.power = 1000000

    def calculate_rpm(self, dt):
        steps_per_rev = 836
        steps = self.encoder.steps
        rpm = (steps / steps_per_rev) * (60 / dt)  # Convert steps per second to RPM
        self.encoder.steps = 0  # Reset the steps for the next calculation
        print(f"Encoder Steps: {steps}, Time Delta: {dt:.2f}, Calculated RPM: {rpm:.2f}")
        return rpm

    def update_motor_power(self, dt):
        rpm = self.calculate_rpm(dt)
        if self.side == "L":
            rpm = -1*rpm
        
        if self.power <= 0.0001 and self.power >= -0.0001:
            return
        
        self.power = self.pid(rpm, dt)
        self.motor.throttle = max(-1, min(self.power + self.motor.throttle, 1))

        print(self.target_rpm)
        print(f"RPM: {rpm:.2f}, PID Output Power: {self.power:.5f}, Motor Throttle: {self.motor.throttle:.2f}")

    def adjust_pid_constants(self, key):
        if key == 'z':
            self.K_P += 0.0001
        elif key == 'x':
            self.K_P = max(0, self.K_P - 0.0001)
        elif key == 'c':
            self.K_I += 0.0001
        elif key == 'v':
            self.K_I = max(0, self.K_I - 0.0001)
        elif key == 'b':
            self.K_D += 0.0001
        elif key == 'n':
            self.K_D = max(0, self.K_D - 0.0001)

        # Update the PID controller with the new constants
        self.pid.tunings = (self.K_P, self.K_I, self.K_D)

    def reset_throttle(self):
        self.motor.throttle = 0


    def update_target_rpm(self, value):
        self.target_rpm += value
        self.pid.setpoint = self.target_rpm
        self.power = 1000000
