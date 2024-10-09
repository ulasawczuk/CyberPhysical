import smbus2
import time
from PIL import Image, ImageDraw, ImageFont

# Configuration
I2C_BUS = 1
OLED_I2C_ADDRESS = 0x3C  # Replace with your OLED address
OLED_WIDTH = 128
OLED_HEIGHT = 64  # Change to 64 if you have a 128x64 display
CHUNK_SIZE = 16  # Maximum data length per transaction

# Initialize the I2C bus
bus = smbus2.SMBus(I2C_BUS)

def init_oled():
    """Initializes the SSD1306 OLED display."""
    commands = [
        0xAE,  # Display Off
        0xD5, 0x80,  # Set Display Clock Divide Ratio / Oscillator Frequency
        0xA8, 0x3F,  # Set Multiplex Ratio (63 rows for 64-height display)
        0xD3, 0x00,  # Set Display Offset
        0x40,  # Set Display Start Line
        0x8D, 0x14,  # Enable Charge Pump
        0x20, 0x00,  # Set Memory Addressing Mode (Horizontal)
        0xA1,  # Segment Re-map
        0xC8,  # COM Output Scan Direction
        0xDA, 0x12,  # Set COM Pins
        0x81, 0xCF,  # Set Contrast Control
        0xD9, 0xF1,  # Set Pre-Charge Period
        0xDB, 0x40,  # Set VCOMH Deselect Level
        0xA4,  # Entire Display ON
        0xA6,  # Set Normal Display
        0xAF,  # Display ON
    ]

    for cmd in commands:
        bus.write_byte_data(OLED_I2C_ADDRESS, 0x00, cmd)

def clear_oled():
    """Clears the OLED display."""
    for page in range(OLED_HEIGHT // 8):
        bus.write_i2c_block_data(OLED_I2C_ADDRESS, 0x00, [0xB0 | page])  # Set page address
        bus.write_i2c_block_data(OLED_I2C_ADDRESS, 0x00, [0x00])  # Set column lower address
        bus.write_i2c_block_data(OLED_I2C_ADDRESS, 0x00, [0x10])  # Set column higher address
        for start in range(0, OLED_WIDTH, CHUNK_SIZE):
            chunk = [0x00] * CHUNK_SIZE
            bus.write_i2c_block_data(OLED_I2C_ADDRESS, 0x40, chunk)

def draw_on_oled():
    """Draws a test pattern on the OLED display."""
    # Create a blank image for the OLED display
    image = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
    draw = ImageDraw.Draw(image)

    # Load a font and draw text on the image
    font = ImageFont.load_default()
    draw.text((0, 0), "Testing", font=font, fill=255)

    # Convert the image to byte data for the OLED
    pixels = image.convert('1').tobytes()

    for page in range(OLED_HEIGHT // 8):
        bus.write_i2c_block_data(OLED_I2C_ADDRESS, 0x00, [0x22, page, page])
        bus.write_i2c_block_data(OLED_I2C_ADDRESS, 0x00, [0x21, 0, OLED_WIDTH - 1])
        bus.write_i2c_block_data(OLED_I2C_ADDRESS, 0x00, [0x10])
        for start in range(0, OLED_WIDTH, CHUNK_SIZE):
            end = start + CHUNK_SIZE
            chunk = pixels[(page * OLED_WIDTH) + start:(page * OLED_WIDTH) + end]
            bus.write_i2c_block_data(OLED_I2C_ADDRESS, 0x40, list(chunk))

def main():
    """Main function to run the display test."""
    init_oled()
    clear_oled()
    draw_on_oled()
    time.sleep(2)  # Display the pattern for 2 seconds

if __name__ == "__main__":
    main()
