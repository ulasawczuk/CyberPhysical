import adafruit_tcs34725

class ColorSensor:

    def __init__(self, i2c_bus):
        self.sensor = adafruit_tcs34725.TCS34725(i2c_bus)
    
    def get_rgb(self):
        # Returns the RGB values as bytes
        return self.sensor.color_rgb_bytes

    def classify_color(self, r, g, b):
        # Classify the color based on the RGB values
        if r > 40 and r > g + b:
            return "Red"
        elif b > 10 and r < 40 and g < 40:
            return "Blue"
        elif r > 40 and g > 40 and b < 5:
            return "Black"
        else:
            return "Unknown"