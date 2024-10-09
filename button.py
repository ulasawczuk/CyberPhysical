import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Pin setup
button_pin = 12  # Button pin
led_pin = 23     # LED pin

# Setup GPIO pins
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Pull-down resistor for the button
GPIO.setup(led_pin, GPIO.OUT)  # LED pin as output

try:
    while True:
        if GPIO.input(button_pin) == GPIO.HIGH:
            # Button is pressed, turn on the LED
            GPIO.output(led_pin, GPIO.HIGH)
            print("Button pressed! LED ON")
        else:
            # Button is not pressed, turn off the LED
            GPIO.output(led_pin, GPIO.LOW)
        
        # Small delay to avoid bouncing issues
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    GPIO.cleanup()  # Clean up all GPIO settings when done
import RPi.GPIO as GPIO
import time

# Set up GPIO mode
GPIO.setmode(GPIO.BCM)

# Pin setup
button_pin = 12  # Button pin
led_pin = 24     # LED pin

# Setup GPIO pins
GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Pull-down resistor for the button
GPIO.setup(led_pin, GPIO.OUT)  # LED pin as output

try:
    while True:
        if GPIO.input(button_pin) == GPIO.HIGH:
            # Button is pressed, turn on the LED
            GPIO.output(led_pin, GPIO.HIGH)
            print("Button pressed! LED ON")
        else:
            # Button is not pressed, turn off the LED
            GPIO.output(led_pin, GPIO.LOW)
        
        # Small delay to avoid bouncing issues
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    GPIO.cleanup()  # Clean up all GPIO settings when done
