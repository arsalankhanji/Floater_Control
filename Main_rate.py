

from Gyro_Sensor import complimentary_imu as Cimu
from Servo_Control import servoMotor 
from time import sleep, time
import csv

sleep_time = 0.0 #0.0001
#dt = 0.01

[bx, by, bz] = Cimu.init_imu() 

servoA = servoMotor.ServoMotor(18)
servoB = servoMotor.ServoMotor(17)
mean = 1500  # corresponding to 90 degrees

phiSetpoint = 0 # degrees
#phiRateSetpoint = 0 # degrees
thetaSetpoint = 0 # degrees

# Outer Control Loop Gains
Kp1 = 1
Ki1 = 0
Kd1 = 0

# Inner Control Loop Gains
Kp2 = -250
Ki2 = 0
Kd2 = 0

# Initialize data logger
logFile = open("data/testData.csv","w",newline="")
csvObj = csv.writer(logFile)

# Initiatng Controller Clock
absStartTime = time()

try:
    previous_errorPhi = 0
    previous_errorPhiRate = 0
    integralPhi = 0
    integralPhiRate = 0
    previousTime = time() - absStartTime
    sleep(0.0) # 1 seconds sleep

    while True:

        # Getting Attitude of Floater
        [Phi , Theta , p , q ] = Cimu.get_imu(bx, by, bz) # Phi about x  &   Theta about y
        # print("Phi: " + str(round(Phi,1)) + " | Theta: " + str(round(Theta,1)) )

        # Computing time delta
        currentTime = time() - absStartTime
        dt = currentTime - previousTime

        # OUTER CONTROL LOOP - Error signal for Attitude Control
        errorPhi = phiSetpoint - Phi
        integralPhi = integralPhi + errorPhi*dt
        derivativePhi = (errorPhi - previous_errorPhi)/dt
        outputPhi = Kp1*errorPhi + Ki1*integralPhi + Kd1*derivativePhi
        previous_errorPhi = errorPhi
        phiRateSetpoint = outputPhi

        # INNER CONTROL LOOP - Error Signal for Rate Controls
        errorPhiRate = phiRateSetpoint - p # error = setpoint - measured_value
        integralPhiRate = integralPhiRate + errorPhiRate*dt
        derivativePhiRate = (errorPhiRate - previous_errorPhiRate)/dt
        outputPhiRate = Kp2*errorPhiRate + Ki2*integralPhiRate + Kd2*derivativePhiRate
        previous_errorPhiRate = errorPhiRate

        # Servo Actuation
        pulseWidthA = mean + outputPhiRate - 125
        #servoA.moveServo(mean + outputPhiRate - 125) #Phi*10)
        servoA.moveServo(pulseWidthA)
        servoB.moveServo(mean + 0*Theta*10 + 120)

        #print("errors >> Kp =" + str(round(outputPhiRate,2)) + " Ki = " + str(round(integralPhiRate,2)) + " Kd = " + str(round(derivativePhiRate,2)) )
        #print("Phi = " + str(round(Phi,2)) + " p = " + str(round(p,2)) + " dt = " + str(round(dt,5)) )
        csvObj.writerow([  round(currentTime,2) , round(Phi,2) , round(p,2) , round(dt,5) , round(pulseWidthA,0)  ])
        previousTime = currentTime

        sleep(sleep_time)
        #sleep(dt)

except:
    servoA.stopServo()
    servoB.stopServo()
    logFile.close()
