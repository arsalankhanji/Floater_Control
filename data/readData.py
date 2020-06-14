
from matplotlib import pyplot as plt
from matplotlib import style
import numpy as np

style.use('ggplot')

time , Phi , p , dt , servoAPulseWidth , PhiFilt = np.loadtxt('testData.csv', unpack = True , delimiter = ',')

fig , axs = plt.subplots(4 , sharex = True)
fig.suptitle('Floater Data Log')
axs[0].plot(time,Phi,time,PhiFilt)
axs[0].set( ylabel = 'Phi' )
axs[1].plot(time,p)
axs[1].set( ylabel = 'p' )
axs[2].plot(time,dt)
axs[2].set(ylabel = 'dt')
axs[3].plot(time,servoAPulseWidth)
axs[3].set(ylabel = 'ServoA Pulse' , xlabel = 'time')

plt.show()
