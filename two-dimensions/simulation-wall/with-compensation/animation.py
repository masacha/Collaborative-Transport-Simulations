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
left = plt.Circle((5, -5), 0.1, fc='g', alpha=0.5)
right = plt.Circle((5, -5), 0.1, fc='r', alpha=0.5)
wall = plt.Circle((0, 1), 0.1, fc='black')
objectpoints = [[-1.0,1.0],[1.0,1.0],[1.0,-1.0],[-1.0,-1.0]]
rectangle = plt.Polygon(objectpoints, closed=True, fc='b')

frame_skip = 100

def init():
    bottom.center = (5, 5)
    left.center = (5, 5)
    right.center = (5, 5)
    ax.add_patch(bottom)
    ax.add_patch(left)
    ax.add_patch(right)
    ax.add_patch(rectangle)
    ax.add_patch(wall)
    return bottom,left,right,rectangle, wall

def animate(i):
    xtopleft = np.cos(data[frame_skip*i][6])*(data[frame_skip*i][4]-1.0)-np.sin(data[frame_skip*i][6])*(data[frame_skip*i][5]+1.0)
    ytopleft = np.sin(data[frame_skip*i][6])*(data[frame_skip*i][4]-1.0)+np.cos(data[frame_skip*i][6])*(data[frame_skip*i][5]+1.0)
    xtopright = np.cos(data[frame_skip*i][6])*(data[frame_skip*i][4]+1.0)-np.sin(data[frame_skip*i][6])*(data[frame_skip*i][5]+1.0)
    ytopright = np.sin(data[frame_skip*i][6])*(data[frame_skip*i][4]+1.0)+np.cos(data[frame_skip*i][6])*(data[frame_skip*i][5]+1.0)
    xbottomright = np.cos(data[frame_skip*i][6])*(data[frame_skip*i][4]+1.0)-np.sin(data[frame_skip*i][6])*(data[frame_skip*i][5]-1.0)
    ybottomright = np.sin(data[frame_skip*i][6])*(data[frame_skip*i][4]+1.0)+np.cos(data[frame_skip*i][6])*(data[frame_skip*i][5]-1.0)
    xbottomleft = np.cos(data[frame_skip*i][6])*(data[frame_skip*i][4]-1.0)-np.sin(data[frame_skip*i][6])*(data[frame_skip*i][5]-1.0)
    ybottomleft = np.sin(data[frame_skip*i][6])*(data[frame_skip*i][4]-1.0)+np.cos(data[frame_skip*i][6])*(data[frame_skip*i][5]-1.0)
    objectpoints = [[xtopleft,ytopleft],[xtopright,ytopright],[xbottomright,ybottomright],[xbottomleft,ybottomleft]]
    rectangle = plt.Polygon(objectpoints, closed=True, fc='b', alpha=0.5)
    ax.add_patch(rectangle)
    bottom.center = (data[frame_skip*i][1], data[frame_skip*i][2])
    left.center = (data[frame_skip*i][10], data[frame_skip*i][11])
    right.center = (data[frame_skip*i][14], data[frame_skip*i][15])
    return bottom, left, right, rectangle, wall,

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=100, 
                               interval=100.0,
                               blit=True)

plt.show()

#Writer = animation.writers['ffmpeg']
#writer = Writer(fps=10)
#anim.save('animation.mp4', writer=writer)

