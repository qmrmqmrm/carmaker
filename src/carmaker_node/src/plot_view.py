#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import os.path as op
import pandas as pd
from config import Config
import matplotlib.pyplot as plt


def make_plot(state, cx, cy, states, target_ind):
	plt.cla()
	# for stopping simulation with the esc key.
	plt.gcf().canvas.mpl_connect(
		'key_release_event',
		lambda event: [exit(0) if event.key == 'escape' else None])
	plot_arrow(state.x, state.y, state.yaw)
	plt.plot(cx, cy, "-r", label="course")
	plt.plot(states.x, states.y, "-b", label="trajectory")
	plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
	plt.axis("equal")
	plt.grid(True)
	plt.title("Speed[km/h]:" + str(state.v)[:4])
	plt.pause(0.001)

	return plt


def all_plot(states, cx, cy):
	plt.cla()
	plt.plot(cx, cy, ".r", label="course")
	plt.plot(states.x, states.y, "-b", label="trajectory")
	plt.legend()
	plt.xlabel("x[m]")
	plt.ylabel("y[m]")
	plt.axis("equal")
	plt.grid(True)

	plt.subplots(1)
	plt.plot(states.t, [iv for iv in states.v], "-r")
	plt.xlabel("Time[s]")
	plt.ylabel("Speed[km/h]")
	plt.grid(True)
	plt.show()

	return plt


def vel_plot(state, states):
	plt.cla()
	plt.gcf().canvas.mpl_connect(
		'key_release_event',
		lambda event: [exit(0) if event.key == 'escape' else None])
	plt.grid(True)
	plt.plot(states.t, [iv for iv in states.v], "-r")
	plt.legend()
	plt.xlabel("Time[s]")
	plt.ylabel("Speed[km/h]")
	plt.axis("equal")
	plt.title("Speed[km/h]:" + str(state.v)[:4])
	plt.grid(True)
	plt.pause(0.00001)

	return plt
