#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

data = np.genfromtxt('collaborative_transport_simulation.dat',
                     names=True,
                     dtype=None,
                     delimiter=' ')

fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 7)

ax = plt.axes(xlim=(-2, 2), ylim=(-2, 2))
bottom = plt.Circle((5, -5), 0.1, fc='y')
objectpoints = [[-1.0,1.0],[1.0,1.0],[1.0,-1.0],[-1.0,-1.0]]
rectangle = plt.Polygon(objectpoints, closed=True, fc='b')

def init():
    bottom.center = (5, 5)
    ax.add_patch(bottom)
    ax.add_patch(rectangle)
    return bottom,rectangle,

def animate(i):
    xtopleft = np.cos(data[1000*i][6])*(data[1000*i][4]-1.0)-np.sin(data[1000*i][6])*(data[1000*i][5]+1.0)
    ytopleft = np.sin(data[1000*i][6])*(data[1000*i][4]-1.0)+np.cos(data[1000*i][6])*(data[1000*i][5]+1.0)
    xtopright = np.cos(data[1000*i][6])*(data[1000*i][4]+1.0)-np.sin(data[1000*i][6])*(data[1000*i][5]+1.0)
    ytopright = np.sin(data[1000*i][6])*(data[1000*i][4]+1.0)+np.cos(data[1000*i][6])*(data[1000*i][5]+1.0)
    xbottomright = np.cos(data[1000*i][6])*(data[1000*i][4]+1.0)-np.sin(data[1000*i][6])*(data[1000*i][5]-1.0)
    ybottomright = np.sin(data[1000*i][6])*(data[1000*i][4]+1.0)+np.cos(data[1000*i][6])*(data[1000*i][5]-1.0)
    xbottomleft = np.cos(data[1000*i][6])*(data[1000*i][4]-1.0)-np.sin(data[1000*i][6])*(data[1000*i][5]-1.0)
    ybottomleft = np.sin(data[1000*i][6])*(data[1000*i][4]-1.0)+np.cos(data[1000*i][6])*(data[1000*i][5]-1.0)
    objectpoints = [[xtopleft,ytopleft],[xtopright,ytopright],[xbottomright,ybottomright],[xbottomleft,ybottomleft]]
    rectangle = plt.Polygon(objectpoints, closed=True, fc='b')
    ax.add_patch(rectangle)
    bottom.center = (data[1000*i][1], data[1000*i][2])
    return bottom,rectangle,

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=100, 
                               interval=100.0,
                               blit=True)

plt.show()

#Writer = animation.writers['ffmpeg']
#writer = Writer(fps=10)
#anim.save('animation.mp4', writer=writer)

