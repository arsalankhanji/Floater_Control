#!/usr/bin/env python

# servo_demo.py
# 2016-10-07
# Public Domain

# servo_demo.py          # Send servo pulses to GPIO 4.
# servo_demo.py 23 24 25 # Send servo pulses to GPIO 23, 24, 25.

import sys
import time
import pigpio

class ServoMotor:

    MIN_WIDTH=500 # Defining Min pulse width in micro-seconds, corresponds to 0 degree angle
    MAX_WIDTH=2500 # Defining Max pulse width in micro-seconds, corresponds to 180 degree angle

    width = [0] # Initializing pulse width array
    used = [False] # Initializing use status of GPIO array

    def __init__(self, G = 18):
        self.pi = pigpio.pi() # connecting to  pi
        if not self.pi.connected:
            exit()
        self.G = G  # e.g. G = [18] # Define array of GPIO pins to control with PWM
        self.used = True
        self.width = 0 # Pulse width of PWM .Range is (MIN_WIDTH, MAX_WIDTH)
        print("Servo Class Initialized...")
    
    def stopServo(self):
        self.pi.set_servo_pulsewidth(self.G, 0)
        self.pi.stop()    

    def moveServo(self, pulseWidth):
        self.pi.set_servo_pulsewidth(self.G, pulseWidth)

if __name__ == '__main__':     # Program entrance
    
    servoA = ServoMotor(18)
    servoA.moveServo(2500)
    #servoA.stopServo()

