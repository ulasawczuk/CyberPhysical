import select
import time
import sys
import tty
import termios
import pwmio
import board
from adafruit_motor import motor
from gpiozero import RotaryEncoder
from simple_pid import PID

# Initialize PWM outputs for motors
pwm1 = pwmio.PWMOut(board.D21, frequency=1000)  # A1
pwm2 = pwmio.PWMOut(board.D16, frequency=1000)  # A2
pwm3 = pwmio.PWMOut(board.D25, frequency=1000)  # B1
pwm4 = pwmio.PWMOut(board.D24, frequency=1000)  # B2

# Initialize DC motors
motorL = motor.DCMotor(pwm1, pwm2)
motorR = motor.DCMotor(pwm3, pwm4)

# Set decay mode
motorL.decay_mode = motor.SLOW_DECAY
motorR.decay_mode = motor.SLOW_DECAY

# Initialize Rotary Encoders (replace the board.Dx with actual GPIO pin numbers)
encL = RotaryEncoder(13, 6)  # Left motor encoder
encR = RotaryEncoder(19, 26)  # Right motor encoder

# PID controllers for each motor
target_rpm = 60  # Target speed in RPM

pidL = PID(Kp=0.2, Ki=0.2, Kd=0.05, setpoint=target_rpm)
pidR = PID(Kp=0.8, Ki=0.2, Kd=0.05, setpoint=target_rpm)

# PID output limits to PWM range [0, 1]
pidL.output_limits = (0, 1)
pidR.output_limits = (0, 1)


# Function to read a single keypress from the terminal
def get_keypress():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Function to convert encoder steps to RPM
def calculate_rpm(encoder, dt):
    steps_per_rev = 20
    steps = encoder.steps
    rpm = (steps / steps_per_rev) * (60 / dt)  # Convert steps per second to RPM
    encoder.steps = 0  # Reset the steps for the next calculation
    return rpm

def control_motors():
    current_time = time.time()
    dt = current_time - control_motors.prev_time  # Time delta in seconds
    control_motors.prev_time = current_time

    # Calculate RPM for both motors
    rpmL = calculate_rpm(encL, dt)
    rpmR = calculate_rpm(encR, dt)

    # Calculate errors
    errorL = target_rpm - rpmL
    errorR = target_rpm - rpmR

    # Accumulate integral of errors
    control_motors.integralL += errorL * dt
    control_motors.integralR += errorR * dt

    # Calculate derivative of errors
    derivativeL = (errorL - control_motors.prev_errorL) / dt
    derivativeR = (errorR - control_motors.prev_errorR) / dt

    # PID control equation using the provided formula
    pwmL = (pidL.Kp * errorL +
            pidL.Ki * control_motors.integralL +
            pidL.Kd * derivativeL)
    
    pwmR = (pidR.Kp * errorR +
            pidR.Ki * control_motors.integralR +
            pidR.Kd * derivativeR)

    # Ensure PWM stays within limits [0, 1]
    pwmL = max(0, min(pwmL, 1))
    pwmR = max(0, min(pwmR, 1))

    # Set motor speeds
    motorL.throttle = pwmL
    motorR.throttle = pwmR

    # Save the current errors for the next derivative calculation
    control_motors.prev_errorL = errorL
    control_motors.prev_errorR = errorR

    # Read encoder positions
    positionL = encL.steps  # Get left encoder position
    positionR = encR.steps  # Get right encoder position

    print(f"Left Encoder Position: {positionL}, Right Encoder Position: {positionR}")
    print(f"RPM L: {rpmL}, RPM R: {rpmR}, PWM L: {pwmL}, PWM R: {pwmR}")

# Initialize previous time and error for control loop
control_motors.prev_time = time.time()
control_motors.prev_errorL = 0
control_motors.prev_errorR = 0
control_motors.integralL = 0
control_motors.integralR = 0

# Function to process motor movement commands
def process_keypress(key):
    if key == 'w':  # Move forward
        print("Moving Forward")
        motorL.throttle = 1.0  # Full speed forward
        motorR.throttle = 1.0  # Full speed forward
    elif key == 's':  # Move backward
        print("Moving Backward")
        motorL.throttle = -1.0  # Full speed backward
        motorR.throttle = -1.0  # Full speed backward
    elif key == 'a':  # Turn left
        print("Turning Left")
        motorL.throttle = -0.5  # Half speed backward
        motorR.throttle = 0.5  # Half speed forward
    elif key == 'd':  # Turn right
        print("Turning Right")
        motorL.throttle = 0.5  # Half speed forward
        motorR.throttle = -0.5  # Half speed backward
    elif key == ' ':  # Stop motors (space bar)
        print("Stopping Motors")
        motorL.throttle = 0.0
        motorR.throttle = 0.0

# Main control loop
try:
    print("Control the motors with 'w' (forward), 's' (backward), 'a' (left), 'd' (right), 'space' (stop), 'q' (quit)")
    running = True
    while running:
        # Check for keyboard input
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = get_keypress()  # Wait for key press
            if key == 'q':  # Quit
                print("Quitting Program")
                motorL.throttle = 0.0
                motorR.throttle = 0.0
                running = False  # Signal to exit loop
            else:
                process_keypress(key)

        # Continuously control motors based on RPM and PID output
        control_motors()

        time.sleep(0.1)  # Add small delay to avoid high CPU usage

except KeyboardInterrupt:
    motorL.throttle = 0.0
    motorR.throttle = 0.0
    print("\nProgram interrupted, motors halted.")
