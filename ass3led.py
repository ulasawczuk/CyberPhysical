import time
import board
import busio
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import math
import digitalio

# Set up the I2C bus and LSM303 sensor
i2c = busio.I2C(board.SCL, board.SDA)
sensor_mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)

led = digitalio.DigitalInOut(board.D25)
led.direction = digitalio.Direction.OUTPUT

# Function to calculate the heading
def get_heading():
    mag_x, mag_y, mag_z = sensor_mag.magnetic
    heading = math.atan2(mag_y, mag_x) * (180 / math.pi)
    if heading < 0:
        heading += 360
    return heading

try:
    while True:
        heading = get_heading()
        print(f"Heading: {heading:.2f} degrees")

        # Check if pointing to the north (e.g., within ±10 degrees)
        if 350 <= heading or heading <= 10:
            led.value = True
        else:
            led.value = False

        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()
