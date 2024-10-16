import select
import socket
import time
import board
from motorController import MotorController
import pwmio
from gpiozero import RotaryEncoder
from adafruit_motor import motor
from simple_pid import PID


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

# Initialize motor controllers for left and right motors
motorL = MotorController(board.D21, board.D16, 13, 6)  # Left motor
motorR = MotorController(board.D25, board.D24, 19, 26)  # Right motor

while True:
    # Accept a connection
    c, addr = s.accept()
    print('Got connection from', addr)
    
    c.send(b'Thank you for connecting')
    last_time = time.time()

    motorL.reset_throttle()
    motorR.reset_throttle()
    
    while True:
        current_time = time.time()
        if current_time - last_time >= 0.2:
            dt = current_time - last_time

            if motorL.power != 0:
                motorL.update_motor_power(dt)
            if motorR.power != 0:
                motorR.update_motor_power(dt)

            last_time = current_time  # Reset control time

            print("-------------------------------")
        
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
                SPEED_INCREMENT = 5 

                if key == 'w':
                    print('Moving forward')
                    motorL.update_target_rpm(SPEED_INCREMENT)
                    motorR.update_target_rpm(SPEED_INCREMENT)
                elif key == 's':
                    print('Moving backward')
                    motorL.update_target_rpm(-1*SPEED_INCREMENT)
                    motorR.update_target_rpm(-1*SPEED_INCREMENT)
                elif key == 'a':  # Turning left
                    motorL.target_rpm = 300
                    motorR.target_rpm = 150
                elif key == 'd':  # Turning right
                    motorL.target_rpm = 150
                    motorR.target_rpm = 300
                elif key == 'q':  # Stop
                    motorL.target_rpm = 0
                    motorR.target_rpm = 0

                # Adjust PID constants based on key press
                motorL.adjust_pid_constants(key)
                motorR.adjust_pid_constants(key)

            except ConnectionResetError:
                print("Client disconnected abruptly.")
                break

        
    # Close the connection with the client
    c.close()
