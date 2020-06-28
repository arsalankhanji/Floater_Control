import board
import busio
import adafruit_bno055

i2c = busio.I2C(board.SCL , board.SDA)

sensor = adafruit_bno055.BNO055_I2C(i2c)

try:
    while(True):
        print(sensor.temperature)
        print(sensor.euler)
        print(sensor.gravity)
except:

