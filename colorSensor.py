import adafruit_tcs34725

class ColorSensor:

    def __init__(self, i2c_bus):
        self.sensor = adafruit_tcs34725.TCS34725(i2c_bus)
    
    def get_rgb(self):
        # Returns the RGB values as bytes
        return self.sensor.color_rgb_bytes

    def classify_color(self, r, g, b):
        # Classify the color based on the RGB values
        if r > 15 and r > g + b:
            return "Red"
        elif g > 15 and g > r + b:
            return "Green"
        elif b > 15 and b > r + g:
            return "Blue"
        elif r > 10 and g > 10 and b < 10:
            return "Yellow"
        elif g > 10 and b > 10 and r < 10:
            return "Cyan"
        elif r > 10 and b > 10 and g < 10:
            return "Magenta"
        elif r > 20 and g > 20 and b > 20:
            return "White"
        elif r < 5 and g < 5 and b < 5:
            return "Black"
        else:
            return "Unknown"