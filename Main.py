from Gyro_Sensor import complimentary_imu as Cimu
from Servo_Control import servoMotor 
from time import sleep, time

sleep_time = 0 #0.0001

[bx, by, bz] = Cimu.init_imu() 

servoA = servoMotor.ServoMotor(18)
servoB = servoMotor.ServoMotor(17)
mean = 1500  # corresponding to 90 degrees

try:
    while True:
        [Phi , Theta ] = Cimu.get_imu(bx, by, bz) # Phi about x  &   Theta about y
        print("Phi: " + str(round(Phi,1)) + " | Theta: " + str(round(Theta,1)) )
        sleep(sleep_time)
        servoA.moveServo(mean + Phi*10)
        servoB.moveServo(mean + Theta*10)

except:
    servoA.stopServo()
    servoB.stopServo()
