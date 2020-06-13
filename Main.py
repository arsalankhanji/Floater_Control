
from Gyro_Sensor import complimentary_imu as Cimu
from Servo_Control import servoMotor 
from time import sleep, time

sleep_time = 0 #0.0001
dt = 0.000001

[bx, by, bz] = Cimu.init_imu() 

servoA = servoMotor.ServoMotor(18)
servoB = servoMotor.ServoMotor(17)
mean = 1500  # corresponding to 90 degrees

phiSetpoint = 0 # degrees
thetaSetpoint = 0 # degrees

# Control Gains
Kp = -250
Ki = 0
Kd = 0

try:
    previous_errorPhi = 0
    integralPhi = 0
    while True:
        # Getting Attitude of Floater
        [Phi , Theta , p , q ] = Cimu.get_imu(bx, by, bz) # Phi about x  &   Theta about y
        # print("Phi: " + str(round(Phi,1)) + " | Theta: " + str(round(Theta,1)) )

        # Error Signal for Controls
        errorPhi = phiSetpoint - p # error = setpoint - measured_value
        integralPhi = integralPhi + errorPhi*dt
        derivativePhi = (errorPhi - previous_errorPhi)/dt
        outputPhi = Kp*errorPhi + Ki*integralPhi + Kd*derivativePhi
        previous_errorPhi = errorPhi

        print("errors >> Kp =" + str(round(outputPhi,2)) + " Ki = " + str(round(integralPhi,2)) + " Kd = " + str(round(derivativePhi,2)) )

	# Servo Actuation
        servoA.moveServo(mean + outputPhi - 125) #Phi*10)
        servoB.moveServo(mean + 0*Theta*10 + 125)

        sleep(sleep_time)
        sleep(dt)

except:
    servoA.stopServo()
    servoB.stopServo()
