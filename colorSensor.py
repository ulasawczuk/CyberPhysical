import adafruit_tcs34725

class ColorSensor:

    def __init__(self, i2c_bus):
        self.sensor = adafruit_tcs34725.TCS34725(i2c_bus)
    
    def get_rgb(self):
        # Returns the RGB values as bytes
        return self.sensor.color_rgb_bytes

    def classify_color(self, r, g, b):
        # Classify the color based on the RGB values
        if r > 20 and r > g + b:
            if r < 40:
                return "Black"
            elif r == 45 and g == 12 and b == 4:
                return "Blue"
            return "Red"
        elif r < 20 and b >= 3 and g > 10 and not r == g == b:
            return "Blue"
        elif r == g or r == g == b or r < 20 and b < 10 and g > 10:
            return "Black"
        else:
            return "Unknown"