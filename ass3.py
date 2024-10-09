import time
import board
import busio
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import math
import digitalio
import adafruit_ssd1306
import adafruit_tcs34725
from PIL import Image, ImageDraw, ImageFont

# Set up the I2C bus and LSM303 sensor
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the magnitude sensor
sensor_mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)

# Initialize the screen
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3C)

# Initialize the color sensor
color_sensor = adafruit_tcs34725.TCS34725(i2c)

# Initialize the LED
led = digitalio.DigitalInOut(board.D25)
led.direction = digitalio.Direction.OUTPUT

# Function to calculate the heading
def get_heading():
    mag_x, mag_y, mag_z = sensor_mag.magnetic
    heading = math.atan2(mag_y, mag_x) * (180 / math.pi)
    if heading < 0:
        heading += 360
    return heading

# Classify color based on RGB values
def classify_color(r, g, b):
    if r > 15 and r > g + b:
        return "Red"
    elif g > 15 and g > r + b:
        return "Green"
    elif b > 15 and b > r + g:
        return "Blue"
    elif r > 10 and g > 10 and b < 10:
        return "Yellow"
    elif g > 10 and b > 10 and r < 10:
        return "Cyan"
    elif r > 10 and b > 10 and g < 10:
        return "Magenta"
    elif r > 20 and g > 20 and b > 20:
        return "White"
    elif r < 5 and g < 5 and b < 5:
        return "Black"
    else:
        return "Unknown"

try:
    while True:
        heading = get_heading()
        print(f"Heading: {heading:.2f} degrees")

        # Check if pointing to the north (e.g., within   10 degrees)
        if 350 <= heading or heading <= 10:
            led.value = True
        else:
            led.value = False

        # Color classification
        r, g, b = color_sensor.color_rgb_bytes
        color_name = classify_color(r, g, b)
    
        # Display the color name on the screen
        image = Image.new("1", (oled.width, oled.height))
        draw = ImageDraw.Draw(image)
    
        # Draw the text
        draw.text((0, 0), f"Color: {color_name}", font=ImageFont.load_default(), fill=255)
    
        # Display the image buffer on the OLED
        oled.image(image)
        oled.show()

        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()
