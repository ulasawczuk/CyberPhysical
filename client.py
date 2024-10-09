import sys
import tty
import termios
import socket

# Create a socket object
s = socket.socket()

# Define the host and port
host = '10.98.212.47'  # IP of Raspberry Pi
port = 12345

def get_key():
    """Reads a single key press from the terminal."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Connect to the server
try:
    s.connect((host, port))
    print(f"Connected to server {host}:{port}")
    
    # Continuously capture key presses
    while True:
        try:
            key = get_key()
            print(f"Key pressed: {key}")
            s.send(key.encode())  # Send the key press to the server
        except KeyboardInterrupt:
            print("\nProgram interrupted. Exiting...")
            break

except socket.error as e:
    print(f"Connection failed. Error: {e}")
finally:
    # Close the socket when done
    s.close()
