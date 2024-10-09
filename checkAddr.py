import smbus2

I2C_BUS = 1  # Default I2C bus number
bus = smbus2.SMBus(I2C_BUS)

def scan_i2c_bus(bus_number):
    """Scan the I2C bus for connected devices and return their addresses."""
    bus = smbus2.SMBus(bus_number)
    devices = []

    for address in range(0x03, 0x78):
        try:
            bus.write_quick(address)
            devices.append(address)
            print(f"Device found at address 0x{address:02X}")
        except IOError:
            pass

    return devices

if __name__ == "__main__":
    print("Scanning I2C bus...")
    found_devices = scan_i2c_bus(I2C_BUS)
    print("Scan complete.")
    if not found_devices:
        print("No I2C devices found.")
    else:
        print("I2C devices found:", [f"0x{addr:02X}" for addr in found_devices])

bus.write_byte_data(0x3C, 0x00, 0xAF)
