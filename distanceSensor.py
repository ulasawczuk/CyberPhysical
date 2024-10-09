import os
import time
import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

# create the spi bus
spi = busio.SPI(clock=board.SCK, MISO=board.MISO, MOSI=board.MOSI)

# create the cs (chip select)
cs = digitalio.DigitalInOut(board.D5)

# create the mcp object
mcp = MCP.MCP3008(spi, cs)

# create an analog input channel on pin 0
chan0 = AnalogIn(mcp, MCP.P0)

print('Raw ADC Value: ', chan0.value)
print('ADC Voltage: ' + str(chan0.voltage) + 'V')




# function to convert voltage to distance (based on the reading I got from my sensor)
def voltage_to_distance(voltage):

    # reference data points (voltage, distance)
    data = [
        (2.66, 15),   # 2.75 V -> 15 cm
        (1.8, 30),    # 2.0 V -> 30 cm
        (1.5, 40),    # 1.5 V -> 40 cm
        (1, 60),    # 1.1 V -> 60 cm
        (0.75, 80),
        (0.67, 90),   # 0.75 V -> 90 cm
        (0.5, 115), 
        (0.43, 130)    # 0.5 V -> 130 cm
    ]

    # if voltage is outside the range, return None
    if voltage > data[0][0] or voltage < data[-1][0]:
        return 0

    # interpolate between the given data points
    for i in range(len(data) - 1):
        v1, d1 = data[i]
        v2, d2 = data[i + 1]

        if v2 <= voltage <= v1:
            # linear interpolation
            distance = d1 + (d2 - d1) * (voltage - v1) / (v2 - v1)
            return distance

    return 0

try:
    while True:
        # read voltage
        voltage = chan0.voltage

        # convert voltage to distance
        distance = voltage_to_distance(voltage)

        print(f"ADC Voltage: {voltage:.2f}V, Distance: {distance:.2f} cm")
        time.sleep(1)

except KeyboardInterrupt:
    print("Measurement stopped by user")





