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

frame_skip = 5000

for i in range(20):

	xtopleft = data[frame_skip*i][1] + np.cos(data[frame_skip*i][3])*(-1.0)-np.sin(data[frame_skip*i][3])*(+1.0)
	ytopleft = data[frame_skip*i][2] + np.sin(data[frame_skip*i][3])*(-1.0)+np.cos(data[frame_skip*i][3])*(+1.0)
	xtopright = data[frame_skip*i][1] + np.cos(data[frame_skip*i][3])*(+1.0)-np.sin(data[frame_skip*i][3])*(+1.0)
	ytopright = data[frame_skip*i][2] + np.sin(data[frame_skip*i][3])*(+1.0)+np.cos(data[frame_skip*i][3])*(+1.0)
	xbottomright = data[frame_skip*i][1] + np.cos(data[frame_skip*i][3])*(+1.0)-np.sin(data[frame_skip*i][3])*(-1.0)
	ybottomright = data[frame_skip*i][2] + np.sin(data[frame_skip*i][3])*(+1.0)+np.cos(data[frame_skip*i][3])*(-1.0)
	xbottomleft = data[frame_skip*i][1] + np.cos(data[frame_skip*i][3])*(-1.0)-np.sin(data[frame_skip*i][3])*(-1.0)
	ybottomleft = data[frame_skip*i][2] + np.sin(data[frame_skip*i][3])*(-1.0)+np.cos(data[frame_skip*i][3])*(-1.0)


	plt.plot([xtopleft, xtopright], [ytopleft,ytopright], color="b", alpha = 0.01+(0.99*i/19))
	plt.plot([xtopright,xbottomright],[ytopright,ybottomright], color = "b", alpha = 0.01+(0.99*i/19))
	plt.plot([xbottomright, xbottomleft],[ybottomright,ybottomright], color = "b", alpha = 0.01+(0.99*i/19))
	plt.plot([xbottomleft,xtopleft],[ybottomleft,ytopleft], color = "b", alpha = 0.01+(0.99*i/19))

for i in range (20-1):
	plt.plot([data[frame_skip*i][4],data[frame_skip*(i+1)][4]],[data[frame_skip*i][5],data[frame_skip*(i+1)][5]], color = 'g', alpha = 0.01+(0.99*i/19))
	plt.plot([data[frame_skip*i][7],data[frame_skip*(i+1)][7]],[data[frame_skip*i][8],data[frame_skip*(i+1)][8]], color = 'pink', alpha = 0.01+(0.99*i/19))
	plt.plot([data[frame_skip*i][10],data[frame_skip*(i+1)][10]],[data[frame_skip*i][11],data[frame_skip*(i+1)][11]], color = 'r', alpha = 0.01+(0.99*i/19))

ax.set_aspect('equal')

plt.show()

