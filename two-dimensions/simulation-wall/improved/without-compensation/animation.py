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
bottom1 = plt.Circle((5, -5), 0.08, fc='y', alpha=0.5)
bottom1front = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
bottom2 = plt.Circle((5, -5), 0.08, fc='g', alpha=0.5)
bottom2front = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
wall = plt.Circle((data[0][13], data[0][14]), 0.05, fc='black')
objectpoints = [[-1.0,1.0],[1.0,1.0],[1.0,-1.0],[-1.0,-1.0]]
rectangle = plt.Polygon(objectpoints, closed=True, fc='b')

frame_skip = 100

def init():
    bottom1.center = (5, 5)
    bottom2.center = (5, 5)
    ax.add_patch(bottom1)
    ax.add_patch(bottom1front)
    ax.add_patch(bottom2)
    ax.add_patch(bottom2front)
    ax.add_patch(rectangle)
    ax.add_patch(wall)
    return bottom1, bottom1front, bottom2, bottom2front, rectangle, wall

def animate(i):
    xtopleft = data[frame_skip*i][1] + np.cos(data[frame_skip*i][3])*(-1.0)-np.sin(data[frame_skip*i][3])*(+1.0)
    ytopleft = data[frame_skip*i][2] + np.sin(data[frame_skip*i][3])*(-1.0)+np.cos(data[frame_skip*i][3])*(+1.0)
    xtopright = data[frame_skip*i][1] + np.cos(data[frame_skip*i][3])*(+1.0)-np.sin(data[frame_skip*i][3])*(+1.0)
    ytopright = data[frame_skip*i][2] + np.sin(data[frame_skip*i][3])*(+1.0)+np.cos(data[frame_skip*i][3])*(+1.0)
    xbottomright = data[frame_skip*i][1] + np.cos(data[frame_skip*i][3])*(+1.0)-np.sin(data[frame_skip*i][3])*(-1.0)
    ybottomright = data[frame_skip*i][2] + np.sin(data[frame_skip*i][3])*(+1.0)+np.cos(data[frame_skip*i][3])*(-1.0)
    xbottomleft = data[frame_skip*i][1] + np.cos(data[frame_skip*i][3])*(-1.0)-np.sin(data[frame_skip*i][3])*(-1.0)
    ybottomleft = data[frame_skip*i][2] + np.sin(data[frame_skip*i][3])*(-1.0)+np.cos(data[frame_skip*i][3])*(-1.0)
    objectpoints = [[xtopleft,ytopleft],[xtopright,ytopright],[xbottomright,ybottomright],[xbottomleft,ybottomleft]]
    rectangle = plt.Polygon(objectpoints, closed=True, fc='b', alpha=0.5)
    ax.add_patch(rectangle)
    bottom1.center = (data[frame_skip*i][4], data[frame_skip*i][5])
    bottom1front.center = (data[frame_skip*i][4]+np.cos(data[frame_skip*i][6])*0.08, data[frame_skip*i][5]+np.sin(data[frame_skip*i][6])*0.08)
    bottom2.center = (data[frame_skip*i][8], data[frame_skip*i][9])
    bottom2front.center = (data[frame_skip*i][8]+np.cos(data[frame_skip*i][10])*0.08, data[frame_skip*i][9]+np.sin(data[frame_skip*i][10])*0.08)
    return bottom1, bottom1front, bottom2, bottom2front, rectangle, wall,

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=100, 
                               interval=100.0,
			       blit = True)

ax.set_aspect('equal')

plt.show()

#Writer = animation.writers['ffmpeg']
#writer = Writer(fps=10)
#anim.save('animation.mp4', writer=writer)

