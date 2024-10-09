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
K_I = 0.1
K_D = 5

# Target speed in RPM
target_rpm = 30  # Default target RPM

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

def calculate_rpm(encoder, dt):
    steps_per_rev = 0.07
    steps = encoder.steps
    rpm = (steps / steps_per_rev) * (60 / dt)  # Convert steps per second to RPM
    encoder.steps = 0  # Reset the steps for the next calculation
    return rpm

def update(l_motor_power, r_motor_power):
    motorL.throttle = l_motor_power
    motorR.throttle = r_motor_power

def calculate_new_power(dt):
    rpmL = calculate_rpm(encL, dt)
    rpmR = calculate_rpm(encR, dt)
    l_power = pidL(rpmL, dt)
    r_power = pidR(rpmR, dt)
    update(l_power, r_power)

    print(f"RPM L: {rpmL:.2f}, RPM R: {rpmR:.2f}, L PID: {l_power:.2f}, R PID: {r_power:.2f}")


# keep track of values for graph
setpoint, y, x = [], [], []

while True:
    # Accept a connection
    c, addr = s.accept()
    print('Got connection from', addr)
    
    c.send(b'Thank you for connecting')
    last_time = time.time()
    
    while True:
        current_time = time.time()
        if current_time - last_time >= 0.5:

            dt = current_time - last_time
            calculate_new_power(dt)  # Update motor control
            last_control_time = current_time  # Reset control time

            x += [current_time - last_time]
            y += [motorL.throttle]
            setpoint += [pidL.setpoint]
        
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
