#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt
from pylab import genfromtxt;  

data = np.genfromtxt('simulation_wall.dat',
                     names=True,
                     dtype=None,
                     delimiter=' ')

x = data[:][0]
y = data[:][7]

fig = plt.figure()

ax1 = fig.add_subplot(111)

ax1.set_title("Reaction Force")    
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Force (N)')

ax1.plot(x,y, c='r', label='the data')

leg = ax1.legend()

plt.savefig('aa.png')
