#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

data = np.genfromtxt('simulation_transport.dat',
                     names=True,
                     dtype=None,
                     delimiter=' ')

fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 7)

ax = plt.axes(xlim=(-2, 4), ylim=(-2, 4))
left1 = plt.Circle((5, -5), 0.08, fc='g', alpha=0.5)
left1front = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
left2 = plt.Circle((5, -5), 0.08, fc='pink', alpha=0.5)
left2front = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
left3 = plt.Circle((5, -5), 0.08, fc='brown', alpha=0.5)
left3front = plt.Circle((5, -5), 0.01, fc='black', alpha=0.5)
wall = plt.Polygon([[data[0][16], data[0][17]],[data[0][16],4]],fc='black')
objectpoints = [[-1.0,1.0],[1.0,1.0],[1.0,-1.0],[-1.0,-1.0]]
rectangle = plt.Polygon(objectpoints, closed=True, fc='b')

frame_skip = 6000

def init():
    ax.add_patch(left1)
    ax.add_patch(left1front)
    ax.add_patch(left2)
    ax.add_patch(left2front)
    ax.add_patch(left3)
    ax.add_patch(left3front)
    ax.add_patch(rectangle)
    ax.add_patch(wall)
    return left1, left1front, left2, left2front, left3, left3front, rectangle, wall,

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
    left1.center = (data[frame_skip*i][4], data[frame_skip*i][5])
    left1front.center = (data[frame_skip*i][4]+np.cos(data[frame_skip*i][6])*0.08, data[frame_skip*i][5]+np.sin(data[frame_skip*i][6])*0.08)
    left2.center = (data[frame_skip*i][8], data[frame_skip*i][9])
    left2front.center = (data[frame_skip*i][8]+np.cos(data[frame_skip*i][10])*0.08, data[frame_skip*i][9]+np.sin(data[frame_skip*i][10])*0.08)
    left3.center = (data[frame_skip*i][18], data[frame_skip*i][19])
    left3front.center = (data[frame_skip*i][18]+np.cos(data[frame_skip*i][20])*0.08, data[frame_skip*i][19]+np.sin(data[frame_skip*i][20])*0.08)
    return left1, left1front, left2, left2front, left3, left3front, rectangle, wall,

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

