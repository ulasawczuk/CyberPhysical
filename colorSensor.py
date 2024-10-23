import adafruit_tcs34725

class ColorSensor:

    def __init__(self, i2c_bus):
        self.sensor = adafruit_tcs34725.TCS34725(i2c_bus)
    
    def get_rgb(self):
        # Returns the RGB values as bytes
        return self.sensor.color_rgb_bytes

    def classify_color(self, r, g, b):
        # Classify the color based on the RGB values
        if r < 10 and g < 10 and b < 10:
            return "Black"
        elif r > 50 and r > g + b:
            return "Red"
        elif g > 50 and g > r + b:
            return "Green"
        elif b > 50 and b > r + g:
            return "Blue"
        elif r > 40 and g > 40 and b < 30:
            return "Yellow"
        elif r > 75 and g > 75 and b > 75:
            return "White"
        else:
            return "Unknown"