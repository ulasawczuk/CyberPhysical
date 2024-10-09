from time import sleep
import board
import adafruit_lsm303dlh_mag
import adafruit_lsm303_accel
import adafruit_tcs34725
i2c = board.I2C()
#sensor = adafruit_tcs34725.TCS34725(i2c)
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
accel = adafruit_lsm303_accel.LSM303_Accel(i2c)

#sensor.integration_time = 200
#sensor.gain = 6
#print('Color: ({0}, {1}, {2})'.format(*sensor.color_rgb_bytes))
#print('Temperature: {0}K'.format(sensor.color_temperature))
#print('Lux: {0}'.format(sensor.lux))


while True:
    print("Acceleration (m/s^2): X=%0.3f Y=%0.3f Z=%0.3f"%accel.acceleration)
    print("Magnetometer (micro-Teslas)): X=%0.3f Y=%0.3f Z=%0.3f"%mag.magnetic)
    print("")
    sleep(0.5)
