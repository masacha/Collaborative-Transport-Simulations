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
bottomfront = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
left1 = plt.Circle((5, -5), 0.1, fc='g', alpha=0.5)
left1front = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
left2 = plt.Circle((5, -5), 0.1, fc='pink', alpha=0.5)
left2front = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
right1 = plt.Circle((5, -5), 0.1, fc='r', alpha=0.5)
right1front = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
right2 = plt.Circle((5, -5), 0.1, fc='brown', alpha=0.5)
right2front = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
wall = plt.Circle((data[0][19], data[0][20]), 0.05, fc='black')
objectpoints = [[-1.0,1.0],[1.0,1.0],[1.0,-1.0],[-1.0,-1.0]]
rectangle = plt.Polygon(objectpoints, closed=True, fc='b')

frame_skip = 100

def init():
    bottom.center = (5, 5)
    left1.center = (5, 5)
    right1.center = (5, 5)
    ax.add_patch(bottom)
    ax.add_patch(bottomfront)
    ax.add_patch(left1)
    ax.add_patch(left1front)
    ax.add_patch(left2)
    ax.add_patch(left2front)
    ax.add_patch(right1)
    ax.add_patch(right1front)
    ax.add_patch(right2)
    ax.add_patch(right2front)
    ax.add_patch(rectangle)
    ax.add_patch(wall)
    return bottom, bottomfront, left1, left1front, left2, left2front, right1, right1front, right2, right2front, rectangle, wall

def animate(i):
    xtopleft = data[frame_skip*i][4] + np.cos(data[frame_skip*i][6])*(-1.0)-np.sin(data[frame_skip*i][6])*(+1.0)
    ytopleft = data[frame_skip*i][5] + np.sin(data[frame_skip*i][6])*(-1.0)+np.cos(data[frame_skip*i][6])*(+1.0)
    xtopright = data[frame_skip*i][4]+ np.cos(data[frame_skip*i][6])*(+1.0)-np.sin(data[frame_skip*i][6])*(+1.0)
    ytopright = data[frame_skip*i][5] + np.sin(data[frame_skip*i][6])*(+1.0)+np.cos(data[frame_skip*i][6])*(+1.0)
    xbottomright = data[frame_skip*i][4]+ np.cos(data[frame_skip*i][6])*(+1.0)-np.sin(data[frame_skip*i][6])*(-1.0)
    ybottomright = data[frame_skip*i][5] + np.sin(data[frame_skip*i][6])*(+1.0)+np.cos(data[frame_skip*i][6])*(-1.0)
    xbottomleft = data[frame_skip*i][4]+ np.cos(data[frame_skip*i][6])*(-1.0)-np.sin(data[frame_skip*i][6])*(-1.0)
    ybottomleft = data[frame_skip*i][5] + np.sin(data[frame_skip*i][6])*(-1.0)+np.cos(data[frame_skip*i][6])*(-1.0)
    objectpoints = [[xtopleft,ytopleft],[xtopright,ytopright],[xbottomright,ybottomright],[xbottomleft,ybottomleft]]
    rectangle = plt.Polygon(objectpoints, closed=True, fc='b', alpha=0.5)
    ax.add_patch(rectangle)
    bottom.center = (data[frame_skip*i][1], data[frame_skip*i][2])
    bottomfront.center = (data[frame_skip*i][1]+np.cos(data[frame_skip*i][3])*0.1, data[frame_skip*i][2]+np.sin(data[frame_skip*i][3])*0.1)
    left1.center = (data[frame_skip*i][10], data[frame_skip*i][11])
    left1front.center = (data[frame_skip*i][10]+np.cos(data[frame_skip*i][12])*0.1, data[frame_skip*i][11]+np.sin(data[frame_skip*i][12])*0.1)
    left2.center = (data[frame_skip*i][21], data[frame_skip*i][22])
    left2front.center = (data[frame_skip*i][21]+np.cos(data[frame_skip*i][23])*0.1, data[frame_skip*i][22]+np.sin(data[frame_skip*i][23])*0.1)
    right1.center = (data[frame_skip*i][14], data[frame_skip*i][15])
    right1front.center = (data[frame_skip*i][14]+np.cos(data[frame_skip*i][16])*0.1, data[frame_skip*i][15]+np.sin(data[frame_skip*i][16])*0.1)
    right2.center = (data[frame_skip*i][25], data[frame_skip*i][26])
    right2front.center = (data[frame_skip*i][25]+np.cos(data[frame_skip*i][27])*0.1, data[frame_skip*i][26]+np.sin(data[frame_skip*i][27])*0.1)
    return bottom, bottomfront, left1, left1front, left2, left2front, right1, right1front, right2, right2front, rectangle, wall,

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

