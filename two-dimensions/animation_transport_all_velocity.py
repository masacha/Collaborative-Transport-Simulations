#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

data = np.genfromtxt('simulation_transport_all_velocity.dat',
                     names=True,
                     dtype=None,
                     delimiter=' ')

fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 7)

ax = plt.axes(xlim=(-2, 6), ylim=(-2, 6))
left1 = plt.Circle((5, -5), 0.1, fc='g', alpha=0.5)
left1front = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
left2 = plt.Circle((5, -5), 0.1, fc='pink', alpha=0.5)
left2front = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
right1 = plt.Circle((5, -5), 0.1, fc='r', alpha=0.5)
right1front = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
objectpoints = [[-1.0,1.0],[1.0,1.0],[1.0,-1.0],[-1.0,-1.0]]
rectangle = plt.Polygon(objectpoints, closed=True, fc='b')

frame_skip = 1000

def init():
    left1.center = (5, 5)
    right1.center = (5, 5)
    ax.add_patch(left1)
    ax.add_patch(left1front)
    ax.add_patch(left2)
    ax.add_patch(left2front)
    ax.add_patch(right1)
    ax.add_patch(right1front)
    ax.add_patch(rectangle)
    return left1, left1front, left2, left2front, right1, right1front, rectangle,

def animate(i):
    xtopleft = data[frame_skip*i][1] + np.cos(data[frame_skip*i][3])*(data[frame_skip*i][1]-1.0)-np.sin(data[frame_skip*i][3])*(data[frame_skip*i][2]+1.0)
    ytopleft = data[frame_skip*i][2] + np.sin(data[frame_skip*i][3])*(data[frame_skip*i][1]-1.0)+np.cos(data[frame_skip*i][3])*(data[frame_skip*i][2]+1.0)
    xtopright = data[frame_skip*i][1] + np.cos(data[frame_skip*i][3])*(data[frame_skip*i][1]+1.0)-np.sin(data[frame_skip*i][3])*(data[frame_skip*i][2]+1.0)
    ytopright = data[frame_skip*i][2] + np.sin(data[frame_skip*i][3])*(data[frame_skip*i][1]+1.0)+np.cos(data[frame_skip*i][3])*(data[frame_skip*i][2]+1.0)
    xbottomright = data[frame_skip*i][1] + np.cos(data[frame_skip*i][3])*(data[frame_skip*i][1]+1.0)-np.sin(data[frame_skip*i][3])*(data[frame_skip*i][2]-1.0)
    ybottomright = data[frame_skip*i][2] + np.sin(data[frame_skip*i][3])*(data[frame_skip*i][1]+1.0)+np.cos(data[frame_skip*i][3])*(data[frame_skip*i][2]-1.0)
    xbottomleft = data[frame_skip*i][1] + np.cos(data[frame_skip*i][3])*(data[frame_skip*i][1]-1.0)-np.sin(data[frame_skip*i][3])*(data[frame_skip*i][2]-1.0)
    ybottomleft = data[frame_skip*i][2] + np.sin(data[frame_skip*i][3])*(data[frame_skip*i][1]-1.0)+np.cos(data[frame_skip*i][3])*(data[frame_skip*i][2]-1.0)
    objectpoints = [[xtopleft,ytopleft],[xtopright,ytopright],[xbottomright,ybottomright],[xbottomleft,ybottomleft]]
    rectangle = plt.Polygon(objectpoints, closed=True, fc='b', alpha=0.5)
    ax.add_patch(rectangle)
    left1.center = (data[frame_skip*i][4], data[frame_skip*i][5])
    left1front.center = (data[frame_skip*i][4]+np.cos(data[frame_skip*i][6])*0.1, data[frame_skip*i][5]+np.sin(data[frame_skip*i][6])*0.1)
    left2.center = (data[frame_skip*i][8], data[frame_skip*i][9])
    left2front.center = (data[frame_skip*i][8]+np.cos(data[frame_skip*i][10])*0.1, data[frame_skip*i][9]+np.sin(data[frame_skip*i][10])*0.1)
    right1.center = (data[frame_skip*i][12], data[frame_skip*i][13])
    right1front.center = (data[frame_skip*i][12]+np.cos(data[frame_skip*i][14])*0.1, data[frame_skip*i][13]+np.sin(data[frame_skip*i][14])*0.1)
    return left1, left1front, left2, left2front, right1, right1front, rectangle,

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=25, 
                               interval=100.0,
			       blit = True)

plt.show()

#Writer = animation.writers['ffmpeg']
#writer = Writer(fps=10)
#anim.save('animation.mp4', writer=writer)

