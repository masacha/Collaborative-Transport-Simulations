#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

data = np.genfromtxt('simulation_wall.dat',
                     names=True,
                     dtype=None,
                     delimiter=' ')

fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 7)

ax = plt.axes(xlim=(-2, 2), ylim=(-2, 2))
bottom = plt.Circle((5, -5), 0.1, fc='y', alpha=0.5)
wall = plt.Circle((data[0][12], data[0][13]), 0.05, fc='black')
objectpoints = [[-1.0,1.0],[1.0,1.0],[1.0,-1.0],[-1.0,-1.0]]
rectangle = plt.Polygon(objectpoints, closed=True, fc='b')
bottomfront = plt.Circle((0, 0), 0.01, fc='black')

frame_skip = 100

def init():
    bottom.center = (5, 5)
    ax.add_patch(bottom)
    ax.add_patch(rectangle)
    ax.add_patch(wall)
    ax.add_patch(bottomfront)
    return bottom,rectangle, bottomfront,

def animate(i):
    xtopleft = data[frame_skip*i][4]+np.cos(data[frame_skip*i][6])*(-1.0)-np.sin(data[frame_skip*i][6])*(1.0)
    ytopleft = data[frame_skip*i][5]+np.sin(data[frame_skip*i][6])*(-1.0)+np.cos(data[frame_skip*i][6])*(1.0)
    xtopright = data[frame_skip*i][4]+np.cos(data[frame_skip*i][6])*(1.0)-np.sin(data[frame_skip*i][6])*(1.0)
    ytopright = data[frame_skip*i][5]+np.sin(data[frame_skip*i][6])*(1.0)+np.cos(data[frame_skip*i][6])*(1.0)
    xbottomright = data[frame_skip*i][4]+np.cos(data[frame_skip*i][6])*(1.0)-np.sin(data[frame_skip*i][6])*(-1.0)
    ybottomright = data[frame_skip*i][5]+np.sin(data[frame_skip*i][6])*(1.0)+np.cos(data[frame_skip*i][6])*(-1.0)
    xbottomleft = data[frame_skip*i][4]+np.cos(data[frame_skip*i][6])*(-1.0)-np.sin(data[frame_skip*i][6])*(-1.0)
    ybottomleft = data[frame_skip*i][5]+np.sin(data[frame_skip*i][6])*(-1.0)+np.cos(data[frame_skip*i][6])*(-1.0)
    objectpoints = [[xtopleft,ytopleft],[xtopright,ytopright],[xbottomright,ybottomright],[xbottomleft,ybottomleft]]
    rectangle = plt.Polygon(objectpoints, closed=True, fc='b', alpha=0.5)
    ax.add_patch(rectangle)
    bottom.center = (data[frame_skip*i][1], data[frame_skip*i][2])
    bottomfront.center = (data[frame_skip*i][1]+np.cos(data[frame_skip*i][3])*0.1, data[frame_skip*i][2]+np.sin(data[frame_skip*i][3])*0.1)
    return bottom,rectangle, bottomfront,

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=100, 
                               interval=100.0,
                               blit=True)

plt.show()

#Writer = animation.writers['ffmpeg']
#writer = Writer(fps=10)
#anim.save('animation.mp4', writer=writer)

