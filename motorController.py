import board
import pwmio
from gpiozero import RotaryEncoder
from adafruit_motor import motor
from simple_pid import PID

class MotorController:
    def __init__(self, pwm1_pin, pwm2_pin, enc_a_pin, enc_b_pin, target_rpm=300):
        # Motor setup
        pwm1 = pwmio.PWMOut(pwm1_pin)
        pwm2 = pwmio.PWMOut(pwm2_pin)
        self.motor = motor.DCMotor(pwm1, pwm2)
        self.motor.decay_mode = motor.SLOW_DECAY

        # Encoder setup
        self.encoder = RotaryEncoder(enc_a_pin, enc_b_pin, max_steps=0)

        # PID constants
        self.K_P = 0.0002
        self.K_I = 0.0002
        self.K_D = 0.0001
        self.pid = PID(self.K_P, self.K_I, self.K_D, setpoint=target_rpm)
        self.pid.output_limits = (-1, 1)

        self.target_rpm = target_rpm

    def calculate_rpm(self, dt):
        steps_per_rev = 70
        steps = self.encoder.steps
        rpm = (steps / steps_per_rev) * (60 / dt)  # Convert steps per second to RPM
        self.encoder.steps = 0  # Reset the steps for the next calculation
        print(f"Calculated RPM: {rpm:.2f} for {dt:.2f}s")
        return rpm

    def update_motor_power(self, dt):
        rpm = self.calculate_rpm(dt)
        power = self.pid(rpm, dt)
        self.motor.throttle = max(-1, min(power + self.motor.throttle, 1))
        print(f"RPM: {rpm:.2f}, PID Output Power: {power:.2f}, Motor Throttle: {self.motor.throttle:.2f}")

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
