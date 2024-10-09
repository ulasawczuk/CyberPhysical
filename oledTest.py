import smbus2
import time
from PIL import Image, ImageDraw, ImageFont

# Configuration
I2C_BUS = 1
OLED_I2C_ADDRESS = 0x3C
OLED_WIDTH = 128
OLED_HEIGHT = 64

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
        0xDA, 0x12,  # Set COM Pins Hardware Configuration
        0x81, 0xCF,  # Set Contrast Control
        0xD9, 0xF1,  # Set Pre-charge Period
        0xDB, 0x40,  # Set VCOMH Deselect Level
        0xA4,  # Entire Display ON
        0xA6,  # Set Normal Display
        0xAF,  # Display ON
    ]
    for cmd in commands:
        bus.write_byte_data(OLED_I2C_ADDRESS, 0x00, cmd)
    print(f"done initoled")

def clear_oled():
    """Clears the OLED display by writing zeros to the entire screen."""
    for page in range(OLED_HEIGHT // 8):
        bus.write_i2c_block_data(OLED_I2C_ADDRESS, 0x00, [0x22, page, page])
        for col in range(0, OLED_WIDTH, 16):
            bus.write_i2c_block_data(OLED_I2C_ADDRESS, 0x40, [0x00] * 16)
    print(f"done clearoled")

def draw_text_on_oled(text):
    """Draws text on the OLED display."""
    # Create a blank image for drawing
    image = Image.new("1", (OLED_WIDTH, OLED_HEIGHT))
    draw = ImageDraw.Draw(image)
    print(f"done blank img")
    text = "Hello, wolrd!"

    # Load a font
    font = ImageFont.load_default()

    # Calculate the position for centered text
    bbox = draw.textbbox((0,0), text, font=font)
    text_width = bbox[2] - bbox[0]
    text_height = bbox[3] - bbox[1]
    x = (OLED_WIDTH - text_width) // 2
    y = (OLED_HEIGHT - text_height) // 2
    print(x)
    print(y)

    # Draw the text
    draw.text((x, y), text, font=font, fill=255)

    # Convert image to data buffer
    buffer = list(image.getdata())

    # Send buffer to OLED
    for page in range(OLED_HEIGHT // 8):
        bus.write_byte_data(OLED_I2C_ADDRESS, 0x00, 0x22)
        bus.write_byte_data(OLED_I2C_ADDRESS, 0x00, page)
        bus.write_byte_data(OLED_I2C_ADDRESS, 0x00, 0x21)
        bus.write_byte_data(OLED_I2C_ADDRESS, 0x00, 0)
        bus.write_byte_data(OLED_I2C_ADDRESS, 0x00, OLED_WIDTH - 1)
        for i in range(OLED_WIDTH // 8):
            bus.write_i2c_block_data(OLED_I2C_ADDRESS, 0x40, buffer[page * OLED_WIDTH + i * 8: page * OLED_WIDTH + (i + 1) * 8])
    print(f"done buffer")

def main():
    """Main function to initialize the OLED and display text."""
    try:
        init_oled()
        clear_oled()
        draw_text_on_oled("Hello World")

        print("Displayed 'Hello World' on the OLED screen.")

    except IOError as e:
        print(f"Failed to communicate with the OLED: {e}")
    except KeyboardInterrupt:
        print("Program stopped by user.")
    finally:
        bus.close()  # Close the I2C bus

if __name__ == "__main__":
    main()
