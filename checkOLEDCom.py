import smbus2

I2C_BUS = 1
OLED_I2C_ADDRESS = 0x3C

bus = smbus2.SMBus(I2C_BUS)

try:
    # Test write: just attempt to send a byte to the display
    bus.write_byte(OLED_I2C_ADDRESS, 0x00)
    print("Successfully communicated with the OLED display.")
except Exception as e:
    print(f"Failed to communicate with the OLED display: {e}")
finally:
    bus.close()
