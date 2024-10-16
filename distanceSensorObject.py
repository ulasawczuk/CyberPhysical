# distanceSensorObject.py

import busio
import digitalio
import board
import adafruit_mcp3xxx.mcp3008 as MCP
from adafruit_mcp3xxx.analog_in import AnalogIn

class DistanceSensor:
    def __init__(self, clock_pin, miso_pin, mosi_pin, cs_pin, adc_channel=MCP.P0):
        # Initialize SPI bus and MCP3008 ADC
        self.spi = busio.SPI(clock=clock_pin, MISO=miso_pin, MOSI=mosi_pin)
        self.cs = digitalio.DigitalInOut(cs_pin)
        self.mcp = MCP.MCP3008(self.spi, self.cs)
        
        # Create an analog input channel (default to channel 0)
        self.channel = AnalogIn(self.mcp, adc_channel)

        # Reference data points for voltage-to-distance conversion
        self.data = [
            (2.66, 15),   # 2.66 V -> 15 cm
            (1.8, 30),    # 1.8 V -> 30 cm
            (1.5, 40),    # 1.5 V -> 40 cm
            (1.0, 60),    # 1.0 V -> 60 cm
            (0.75, 80),   # 0.75 V -> 80 cm
            (0.67, 90),   # 0.67 V -> 90 cm
            (0.5, 115),   # 0.5 V -> 115 cm
            (0.43, 130)   # 0.43 V -> 130 cm
        ]

    def voltage_to_distance(self, voltage):
        """Converts voltage from the sensor to distance using linear interpolation."""
        if voltage > self.data[0][0] or voltage < self.data[-1][0]:
            return 0  # Return 0 if voltage is outside range

        # Interpolate between the reference data points
        for i in range(len(self.data) - 1):
            v1, d1 = self.data[i]
            v2, d2 = self.data[i + 1]

            if v2 <= voltage <= v1:
                # Linear interpolation formula
                distance = d1 + (d2 - d1) * (voltage - v1) / (v2 - v1)
                return distance

        return 0  # Return 0 if no match found

    def read_distance(self):
        """Reads the current voltage and returns the corresponding distance."""
        voltage = self.channel.voltage
        distance = self.voltage_to_distance(voltage)
        return voltage, distance
