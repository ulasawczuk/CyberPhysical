import select
import socket
import time
import busio
import board
from distanceSensorObject import DistanceSensor
from motorController import MotorController
from colorSensor import ColorSensor
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

STOP_DISTANCE = 30  # cm
RESUME_DISTANCE = 40
MOTOR_SPEED = 45
VALUE = MOTOR_SPEED
motor_stopped = False

# Initialize motor controllers for left and right motors
motorL = MotorController(board.D21, board.D16, 19, 26, 1)  # Left motor
motorR = MotorController(board.D25, board.D24, 13, 6, 1)  # Right motor


distanceSensor = DistanceSensor(clock_pin=board.SCK, miso_pin=board.MISO, mosi_pin=board.MOSI, cs_pin=board.D22)
controlDistance = True

i2c = busio.I2C(board.SCL, board.SDA)
colorSensor = ColorSensor(i2c)
followLine = True
colorsDiffer = False
turning_left = False
turning_right = False


while True:
    # Accept a connection
    c, addr = s.accept()
    print('Got connection from', addr)
    
    c.send(b'Thank you for connecting')
    last_time = time.time()

    #motorL.reset_throttle()
    #motorR.reset_throttle()

    correct_color = "Black"
    found = False

    error = False

    halfSecondColor = 0
    halfSecondDistance = 0
    
    if followLine:
        try:
            r, g, b = colorSensor.get_rgb()
        except OSError as e:
            print("I2C error, retrying in 0.1 seconds:", e)
            #time.sleep(0.1)  # Small delay before retrying
            continue
        current_color = colorSensor.classify_color(r, g, b)
        print(f"red: {r}, green: {g}, blue: {b}")  
        print(f"Color: "+ current_color)
    
    while True:
        current_time = time.time()
        if current_time - last_time >= 0.2:
            dt = current_time - last_time
            halfSecondColor += dt
            halfSecondDistance += dt
            print(halfSecondColor)
            
            #if current_color == "Black":
                #motorL.update_motor_power(dt)
                #motorR.update_motor_power(dt)

            # HANDLING DISTANCE

            if controlDistance and halfSecondDistance >= 0.3 and halfSecondDistance <= 0.45 or controlDistance and halfSecondDistance >= 0.5:
                voltage, distance = distanceSensor.read_distance()
                print(f"ADC Voltage: {voltage:.2f}V, Distance: {distance:.2f} cm")
                halfSecondDistance = 0

                if distance <= STOP_DISTANCE and not motor_stopped and distance != 0 and motorR.target_rpm != 0:
                    # Stop motors if the object is too close
                    print("Object detected within stop distance, stopping motors.")
                    #motorL.update_target_rpm(-15)
                    #motorR.update_target_rpm(-15)
                    motorL.motor.throttle = -0.3
                    motorR.motor.throttle = -0.3
                    motor_stopped = True
                    followLine = False

                elif distance >= RESUME_DISTANCE and motor_stopped or distance == 0 and motor_stopped:
                    # Resume motors if object is far enough
                    print("Object far enough or avoided, resuming motors.")
                    #motorL.update_target_rpm(MOTOR_SPEED)
                    #motorR.update_target_rpm(MOTOR_SPEED)
                    motorL.motor.throttle = 0.3
                    motorR.motor.throttle = 0.3
                    motor_stopped = False
                    followLine = True


            # HANDLING COLOR

            if followLine:
                
                # Sampling color every 0.4 second to avoid i2c error
                if halfSecondColor >= 0.3 and halfSecondColor <= 0.45 or halfSecondColor >= 0.5:
                    try:
                        r, g, b = colorSensor.get_rgb()
                        error = False
                    except OSError as e:   
                        print("I2C error, retrying in 0.1 seconds:", e)
                        #time.sleep(0.1)  # Small delay before retrying
                        halfSecondColor = 0
                        error = True
                        continue
                    current_color = colorSensor.classify_color(r, g, b)
                    print(f"red: {r}, green: {g}, blue: {b}")  
                    print(f"Color: "+ current_color)
                    if error:
                        current_color = "Stop" # stop if there was an error
                    halfSecondColor = 0  

                if current_color == "Stop":
                    #motorL.update_target_rpm(0)
                    #motorR.update_target_rpm(0)
                    motorL.motor.throttle = 0
                    motorR.motor.throttle = 0

                if current_color == "Black" and (turning_left or turning_right):
                    # Stop turning, resume forward motion
                    if turning_left:
                        #motorL.update_target_rpm(20)
                        #motorR.update_target_rpm(8)
                        motorL.motor.throttle = 0.5
                        motorR.motor.throttle = 0.2
                    if turning_right:
                        #motorL.update_target_rpm(8)
                        #motorR.update_target_rpm(20)
                        motorL.motor.throttle = 0.2
                        motorR.motor.throttle = 0.5
                    turning_left = False
                    turning_right = False
                    found = True
                    print("Back on black tape, turning a bit.")

                elif current_color == "Black" and not turning_left and not turning_right and found:
                    #motorL.update_target_rpm(MOTOR_SPEED)
                    #motorR.update_target_rpm(MOTOR_SPEED)
                    motorL.motor.throttle = 0.3
                    motorR.motor.throttle = 0.3
                    found = False
                    print("Back on black tape, moving straight.")

                # If red is detected, turn right to find black
                elif current_color == "Red" and not turning_right:
                    print("Red tape detected, turning right.")
                    #motorL.update_target_rpm(24)  
                    #motorR.update_target_rpm(9) 
                    motorL.motor.throttle = 0.2
                    motorR.motor.throttle = -0.1
                    turning_right = True  
                    turning_left = False 
                    #halfSecondColor = 0.1

                # If blue is detected, turn left to find black
                elif current_color == "Blue" and not turning_left:
                    print("Blue tape detected, turning left.")
                    #motorL.update_target_rpm(9) 
                    #motorR.update_target_rpm(24)  
                    motorL.motor.throttle = -0.1
                    motorR.motor.throttle = 0.2
                    turning_left = True  
                    turning_right = False
                    #halfSecondColor = 0.1

                
            last_time = current_time 

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
                VALUE = VALUE + SPEED_INCREMENT

                if key == 'w':
                    print('Moving forward')
                    motorL.motor.throttle = 0.7
                    motorR.motor.throttle = 0.7
                elif key == 's':
                    print('Moving backward')
                    motorL.motor.throttle = -0.7
                    motorR.motor.throttle = -0.7
                elif key == 'a':  # Turning left
                    motorL.motor.throttle = 0
                    motorR.motor.throttle = 0.5
                elif key == 'd':  # Turning right
                    motorL.motor.throttle = 0.5
                    motorR.motor.throttle = 0
                elif key == 'q':  # Stop
                    motorL.motor.throttle = 0
                    motorR.motor.throttle = 0
                elif key == 'f':
                    t = followLine
                    followLine = not t
                elif key == 'g':
                    t = followLine
                    controlDistance = not t
                elif key == 'u':
                    t = update
                    update = not t
                    print(update)
                

                # Adjust PID constants based on key press
                motorL.adjust_pid_constants(key)
                motorR.adjust_pid_constants(key) 

            except ConnectionResetError:
                print("Client disconnected abruptly.")
                break

        
    # Close the connection with the client
    c.close()
