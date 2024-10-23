import adafruit_tcs34725

class ColorSensor:

    def __init__(self, i2c_bus):
        self.sensor = adafruit_tcs34725.TCS34725(i2c_bus)
    
    def get_rgb(self):
        # Returns the RGB values as bytes
        return self.sensor.color_rgb_bytes

    def classify_color(self, r, g, b):
        # Classify the color based on the RGB values
        if r > 80 and r < 200 and r > g + b:
            return "Red"
        elif g > 20 and g > r + b:
            return "Green"
        elif b > 20 and b + 20 > r + g:
            return "Blue"
        elif r > 10 and g > 10 and b < 10:
            return "Yellow"
        elif r > 20 and r < 200 and g < 10 and b < 5:
            return "White"
        elif r > 200 and g < 5 and b < 5:
            return "Black"
        else:
            return "Unknown"