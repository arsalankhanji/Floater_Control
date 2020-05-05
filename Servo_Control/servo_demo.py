#!/usr/bin/env python

# servo_demo.py
# 2016-10-07
# Public Domain

# servo_demo.py          # Send servo pulses to GPIO 4.
# servo_demo.py 23 24 25 # Send servo pulses to GPIO 23, 24, 25.

import sys
import time
import random
import pigpio

NUM_GPIO=32 # Total number of GPIO that can be controlled

MIN_WIDTH=500 # Defining Min pulse width in micro-seconds, corresponds to 0 degree angle
MAX_WIDTH=2500 # Defining Max pulse width in micro-seconds, corresponds to 180 degree angle

step = [0]*NUM_GPIO  # Initializing pulse increment/decrement steps array
width = [0]*NUM_GPIO # Initializing pulse width array
used = [False]*NUM_GPIO # Initializing use status of GPIO array

pi = pigpio.pi() # connecting to  pi

if not pi.connected:
   exit()

#if len(sys.argv) == 1:
#   G = [18]
#else:
#   G = []
#   for a in sys.argv[1:]:
#      G.append(int(a))
G = [18] # Define array of GPIO pins to control with PWM

for g in G:
   used[g] = True
   step[g] = 500 # random.randrange(5, 25)
   if step[g] % 2 == 0:  # if step[g] is an even number then results will be 1 other wise it will be zero
      step[g] = -step[g]
   width[g] = 0 # random.randrange(MIN_WIDTH, MAX_WIDTH+1)

print("Sending servos pulses to GPIO {}, control C to stop.".
   format(' '.join(str(g) for g in G)))

while True:

   try:

      for g in G:

         pi.set_servo_pulsewidth(g, width[g])

         print(g, width[g] , step[g])

         width[g] += step[g]

         if width[g]<MIN_WIDTH or width[g]>MAX_WIDTH:
            step[g] = -step[g]
            width[g] += step[g]

      time.sleep(0.5)

   except KeyboardInterrupt:
      break

print("\nTidying up")

for g in G:
   pi.set_servo_pulsewidth(g, 0)

pi.stop()

