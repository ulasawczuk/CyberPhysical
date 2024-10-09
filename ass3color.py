import board
import busio
import adafruit_ssd1306
import adafruit_tcs34725
import time
from PIL import Image, ImageDraw, ImageFont

# Initialize I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Initialize the screen
oled = adafruit_ssd1306.SSD1306_I2C(128, 64, i2c, addr=0x3C)

# Initialize the color sensor
color_sensor = adafruit_tcs34725.TCS34725(i2c)

# Classify color based on RGB values
def classify_color(r, g, b):
    if r > g and r > b:
        return "Red"
    elif g > r and g > b:
        return "Green"
    elif b > r and b > g:
        return "Blue"
    else:
        return "Unknown"

while True:

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
    # Small delay to reduce the loop frequency
    time.sleep(0.1)
