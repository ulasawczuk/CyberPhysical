import select
import socket
import time
import board
import pwmio
from gpiozero import RotaryEncoder
from adafruit_motor import motor
from simple_pid import PID

# Motor setup
pwm1 = pwmio.PWMOut(board.D21)  # A1
pwm2 = pwmio.PWMOut(board.D16)  # A2
pwm3 = pwmio.PWMOut(board.D25)  # B1
pwm4 = pwmio.PWMOut(board.D24)  # B2

motorL = motor.DCMotor(pwm1, pwm2)
motorR = motor.DCMotor(pwm3, pwm4)

motorL.decay_mode = motor.SLOW_DECAY
motorR.decay_mode = motor.SLOW_DECAY

# Encoder setup
encL = RotaryEncoder(13, 6)  # Left motor encoder (replace with actual GPIO pins)
encR = RotaryEncoder(19, 26) 

# PID constants
K_P = 5
K_I = 0.2
K_D = 0.05

# Target speed in RPM
target_rpm = 400  # Default target RPM

# Initialize PID controllers for both motors
pidL = PID(K_P, K_I, K_D, setpoint=target_rpm)
pidR = PID(K_P, K_I, K_D, setpoint=target_rpm)

# Set output limits for the PID to match the motor throttle range [0, 1]
pidL.output_limits = (0, 1)
pidR.output_limits = (0, 1)

# Create socket
s = socket.socket()

# Host and port
host = '10.98.212.47'  # IP of Raspberry Pi
port = 12345

try:
    s.bind((host, port))
    print(f'Server started on {host}:{port}')
except socket.error as e:
    print(f"Bind failed. Error: {e}")
    exit()

# Max 5 pending connections
s.listen(5)


# Function to calculate RPM from encoder readings
def calculate_rpm(encoder, dt):
    steps_per_rev = 700
    steps = encoder.steps
    rpm = (steps / steps_per_rev) * (60 / dt)  # Convert steps per second to RPM
    encoder.steps = 0  # Reset the steps for the next calculation
    return rpm

# Function to manually compute the control effort using the provided PID formula
def manual_pid_control(error, integral, previous_error, dt):
    integral += error * dt
    derivative = (error - previous_error) / dt
    control_effort = K_P * error + K_I * integral + K_D * derivative
    return control_effort, integral, error

# Function to control motors using both PID and manual calculation
def control_motors():
    current_time = time.time()
    dt = current_time - control_motors.prev_time
    control_motors.prev_time = current_time

    # Calculate RPM for both motors
    rpmL = calculate_rpm(encL, dt)
    rpmR = calculate_rpm(encR, dt)

    # Update PID controllers with current RPM feedback
    pidL.setpoint = target_rpm
    pidR.setpoint = target_rpm

    # Compute the control effort from the PID controllers
    controlL_pid = pidL(rpmL)
    controlR_pid = pidR(rpmR)

    # Calculate errors for manual PID control
    errorL = target_rpm - rpmL
    errorR = target_rpm - rpmR

    # Manual PID control for left motor
    global integralL, previous_errorL
    controlL_manual, integralL, previous_errorL = manual_pid_control(errorL, integralL, previous_errorL, dt)
    controlL_manual = max(0, min(controlL_manual, 1))  # Limit control effort to [0, 1]

    # Manual PID control for right motor
    global integralR, previous_errorR
    controlR_manual, integralR, previous_errorR = manual_pid_control(errorR, integralR, previous_errorR, dt)
    controlR_manual = max(0, min(controlR_manual, 1))  # Limit control effort to [0, 1]

    # Combine PID and manual control efforts (for demonstration)
    motorL.throttle = (controlL_pid + controlL_manual) / 2
    motorR.throttle = (controlR_pid + controlR_manual) / 2

    # Print feedback for debugging
    print(f"RPM L: {rpmL}, RPM R: {rpmR}, Control L PID: {controlL_pid}, Control R PID: {controlR_pid}")
    print(f"Control L Manual: {controlL_manual}, Control R Manual: {controlR_manual}")

# Initialize previous time for PID control
control_motors.prev_time = time.time()

# Initialize integral and previous error for manual PID
integralL = 0
previous_errorL = 0
integralR = 0
previous_errorR = 0

# keep track of values for graph
setpoint, y, x = [], [], []

while True:
    # Accept a connection
    c, addr = s.accept()
    print('Got connection from', addr)
    
    c.send(b'Thank you for connecting')
    last_control_time = time.time()
    
    while True:
        current_time = time.time()
        if current_time - last_control_time >= 0.2:
            control_motors()  # Update motor control
            last_control_time = current_time  # Reset control time
        
        # Use select to check if there's data to read from the client
        ready_to_read, _, _ = select.select([c], [], [], 0.1)  # Wait for 0.1 seconds
        
        for sock in ready_to_read:
            try:
                # Receive key press data from the client (non-blocking)
                key_data = sock.recv(1024)

                if not key_data:
                    # If no data is received, the client has disconnected
                    print("Client disconnected.")
                    break

                # Decode the received data from bytes to string
                key = key_data.decode().strip()
                print(f"Key received from client: {key}")

                # Different keys control the target speed
                if key == 'w':
                    print('Moving forward')
                    target_rpm = 30
                elif key == 's':
                    print('Moving backward')
                    target_rpm = -30
                elif key == 'a':  # Turning left
                    target_rpm = 15
                elif key == 'd':  # Turning right
                    target_rpm = 15
                elif key == 'b':  # Stop
                    target_rpm = 0

            except ConnectionResetError:
                print("Client disconnected abruptly.")
                break
        
    # Close the connection with the client
    c.close()
