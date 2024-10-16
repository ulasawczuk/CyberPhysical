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
encL = RotaryEncoder(13, 6, max_steps = 0)  # Left motor encoder (replace with actual GPIO pins)
encR = RotaryEncoder(19, 26, max_steps = 0) 

# PID constants
K_P = 0.0007
K_I = 0
K_D = 0.0001

# Target speed in RPM
target_rpmL = 0  # Default target RPM
target_rpmR = 0

# Initialize PID controllers for both motors
pidL = PID(K_P, K_I, K_D, setpoint=target_rpmL)
pidR = PID(K_P, K_I, K_D, setpoint=target_rpmR)

# Set output limits for the PID to match the motor throttle range [0, 1]
pidL.output_limits = (-1, 1)
pidR.output_limits = (-1, 1)

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

# def get_steps_p_sam(dt, lprevsteps, rprevsteps):
#     l_speed = (encL.steps - lprevsteps) / (dt)
#     r_speed = (encR.steps - rprevsteps) / (dt)
#     print(f"steps L: {encL.steps:.2f}, steps R: {encR.steps:.2f}")
#     return l_speed, r_speed

def calculate_rpm(encoder, dt):
    steps_per_rev = 70
    steps = encoder.steps
    print(f"dt:  {dt:.2f}")
    rpm = (steps / steps_per_rev) * (60 / dt)  # Convert steps per second to RPM
    encoder.steps = 0  # Reset the steps for the next calculation
    return rpm

def update(l_motor_power, r_motor_power):
    motorL.throttle = max(-1, min(l_motor_power + motorL.throttle, 1))
    motorR.throttle = max(-1, min(r_motor_power + motorR.throttle, 1))

def calculate_new_power(dt, lprevsteps, rprevsteps):
    rpmL = calculate_rpm(encL, dt) * -1
    rpmR = calculate_rpm(encR, dt)
    # rpmL, rpmR = get_steps_p_sam(dt, lprevsteps, rprevsteps)
    l_power = pidL(rpmL, dt)
    r_power = pidR(rpmR, dt)
    update(l_power, r_power)

    print(f"RPM L: {rpmL:.2f}, RPM R: {rpmR:.2f}, L PID: {l_power:.2f}, R PID: {r_power:.2f}")


def adjust_pid_constants(key):
    global K_P, K_I, K_D

    if key == 'z':
        K_P += 0.0001
        print(f"K_P increased to {K_P:.4f}")
    elif key == 'x':
        K_P = max(0, K_P - 0.0001)
        print(f"K_P decreased to {K_P:.4f}")
    elif key == 'c':
        K_I += 0.0001
        print(f"K_I increased to {K_I:.4f}")
    elif key == 'v':
        K_I = max(0, K_I - 0.0001)
        print(f"K_I decreased to {K_I:.4f}")
    elif key == 'b':
        K_D += 0.0001
        print(f"K_D increased to {K_D:.4f}")
    elif key == 'n':
        K_D = max(0, K_D - 0.0001)
        print(f"K_D decreased to {K_D:.4f}")

    # Update the PID controllers with the new constants
    pidL.tunings = (K_P, K_I, K_D)
    pidR.tunings = (K_P, K_I, K_D)

# keep track of values for graph
setpoint, y, x = [], [], []

while True:
    # Accept a connection
    c, addr = s.accept()
    print('Got connection from', addr)
    
    c.send(b'Thank you for connecting')
    last_time = time.time()
    lprevsteps = 0
    rprevsteps = 0

    motorL.throttle = 0
    motorR.throttle = 0
    
    while True:
        current_time = time.time()
        if current_time - last_time >= 0.2:

            dt = current_time - last_time
            calculate_new_power(dt, lprevsteps, rprevsteps)  # Update motor control
            last_time = current_time  # Reset control time

            x += [current_time - last_time]
            y += [motorL.throttle]
            setpoint += [pidL.setpoint]

            lprevsteps = encL.steps
            rprevsteps = encR.steps
        
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
                    target_rpmL = target_rpmL + 10
                    target_rpmR = target_rpmR + 10
                elif key == 's':
                    print('Moving backward')
                    target_rpmL = (-1 * target_rpmL) - 10
                    target_rpmR = (-1 * target_rpmR) - 10
                elif key == 'a':  # Turning left
                    target_rpmL = 30
                    target_rpmR = 15
                elif key == 'd':  # Turning right
                    target_rpmL = 15
                    target_rpmR = 30
                elif key == 'q':  # Stop
                    target_rpm = 0

                pidL = PID(K_P, K_I, K_D, setpoint=target_rpmL)
                pidR = PID(K_P, K_I, K_D, setpoint=target_rpmR)
                
                adjust_pid_constants(key)

            except ConnectionResetError:
                print("Client disconnected abruptly.")
                break
        
    # Close the connection with the client
    c.close()
