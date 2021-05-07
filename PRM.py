# -*- coding: utf-8 -*-
"""
Created on Wed Jan 20 17:47:08 2021

@author: GSS-fearless
"""

"""
Probabilistic Road Map (PRM) Planner
author: Atsushi Sakai (@Atsushi_twi)
"""
import random
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
import time

# parameter
N_SAMPLE = 600  # number of sample_points
N_KNN = 30  # number of edge from one sampled point
MAX_EDGE_LEN = 30.0  # [m] Maximum edge length
show_animation = True
total_time = []

class Node:
	"""
	Node class for dijkstra search
	"""

	def __init__(self, x, y, cost, parent_index):
		self.x = x
		self.y = y
		self.cost = cost
		self.parent_index = parent_index

	def __str__(self):
		return str(self.x) + "," + str(self.y) + "," + \
			   str(self.cost) + "," + str(self.parent_index)


def prm_planning(sx, sy, gx, gy, ox, oy, rr):
	obstacle_kd_tree = cKDTree(np.vstack((ox, oy)).T)
	sample_x, sample_y = sample_points(sx, sy, gx, gy,
									   rr, ox, oy, obstacle_kd_tree)
	if show_animation:
		plt.plot(sample_x, sample_y, ".b")
	road_map = generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree)
	rx, ry = dijkstra_planning(												# This is Dijkstra planning
		sx, sy, gx, gy, road_map, sample_x, sample_y)
	return rx, ry


def is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree):
	x = sx
	y = sy
	dx = gx - sx
	dy = gy - sy
	yaw = math.atan2(gy - sy, gx - sx)
	d = math.hypot(dx, dy)
	if d >= MAX_EDGE_LEN:
		return True
	D = rr
	n_step = round(d / D)
	for i in range(n_step):
		dist, _ = obstacle_kd_tree.query([x, y])
		if dist <= rr:
			return True  # collision
		x += D * math.cos(yaw)
		y += D * math.sin(yaw)
	# goal point check
	dist, _ = obstacle_kd_tree.query([gx, gy])
	if dist <= rr:
		return True  # collision
	return False  # OK


def generate_road_map(sample_x, sample_y, rr, obstacle_kd_tree):
	"""
	Road map generation
	sample_x: [m] x positions of sampled points
	sample_y: [m] y positions of sampled points
	rr: Robot Radius[m]
	obstacle_kd_tree: KDTree object of obstacles
	"""
	road_map = []
	n_sample = len(sample_x)
	sample_kd_tree = cKDTree(np.vstack((sample_x, sample_y)).T)
	for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):
		dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
		edge_id = []
		for ii in range(1, len(indexes)):
			nx = sample_x[indexes[ii]]
			ny = sample_y[indexes[ii]]
			if not is_collision(ix, iy, nx, ny, rr, obstacle_kd_tree):
				edge_id.append(indexes[ii])
			if len(edge_id) >= N_KNN:
				break
		road_map.append(edge_id)
	#  plot_road_map(road_map, sample_x, sample_y)
	return road_map


def dijkstra_planning(sx, sy, gx, gy, road_map, sample_x, sample_y):
	"""
	s_x: start x position [m]
	s_y: start y position [m]
	gx: goal x position [m]
	gy: goal y position [m]
	ox: x position list of Obstacles [m]
	oy: y position list of Obstacles [m]
	rr: robot radius [m]
	road_map: ??? [m]
	sample_x: ??? [m]
	sample_y: ??? [m]
	@return: Two lists of path coordinates ([x1, x2, ...], [y1, y2, ...]), empty list when no path was found
	"""
	start_node = Node(sx, sy, 0.0, -1)
	goal_node = Node(gx, gy, 0.0, -1)
	open_set, closed_set = dict(), dict()
	open_set[len(road_map) - 2] = start_node
	path_found = True
	while True:
		if not open_set:
			print("Cannot find path")
			path_found = False
			break
		c_id = min(open_set, key=lambda o: open_set[o].cost)
		current = open_set[c_id]
		# show graph
		if show_animation and len(closed_set.keys()) % 2 == 0:
			# for stopping simulation with the esc key.
			plt.gcf().canvas.mpl_connect(
				'key_release_event',
				lambda event: [exit(0) if event.key == 'escape' else None])
			plt.plot(current.x, current.y, "xg")
			plt.pause(0.001)
		if c_id == (len(road_map) - 1):
			print("goal is found!")
			goal_node.parent_index = current.parent_index
			goal_node.cost = current.cost
			break
		# Remove the item from the open set
		del open_set[c_id]
		# Add it to the closed set
		closed_set[c_id] = current
		# expand search grid based on motion model
		for i in range(len(road_map[c_id])):
			n_id = road_map[c_id][i]
			dx = sample_x[n_id] - current.x
			dy = sample_y[n_id] - current.y
			d = math.hypot(dx, dy)
			node = Node(sample_x[n_id], sample_y[n_id],
						current.cost + d, c_id)
			if n_id in closed_set:
				continue
			# Otherwise if it is already in the open set
			if n_id in open_set:
				if open_set[n_id].cost > node.cost:
					open_set[n_id].cost = node.cost
					open_set[n_id].parent_index = c_id
			else:
				open_set[n_id] = node
	if path_found is False:
		return [], []
	# generate final course
	rx, ry = [goal_node.x], [goal_node.y]
	parent_index = goal_node.parent_index
	while parent_index != -1:
		n = closed_set[parent_index]
		rx.append(n.x)
		ry.append(n.y)
		parent_index = n.parent_index
	return rx, ry


def plot_road_map(road_map, sample_x, sample_y):  # pragma: no cover
	for i, _ in enumerate(road_map):
		for ii in range(len(road_map[i])):
			ind = road_map[i][ii]
			plt.plot([sample_x[i], sample_x[ind]],
					 [sample_y[i], sample_y[ind]], "-k")


def sample_points(sx, sy, gx, gy, rr, ox, oy, obstacle_kd_tree):
	max_x = max(ox)
	max_y = max(oy)
	min_x = min(ox)
	min_y = min(oy)
	sample_x, sample_y = [], []
	while len(sample_x) <= N_SAMPLE:
		tx = (random.random() * (max_x - min_x)) + min_x
		ty = (random.random() * (max_y - min_y)) + min_y
		dist, index = obstacle_kd_tree.query([tx, ty])
		if dist >= rr:
			sample_x.append(tx)
			sample_y.append(ty)
	sample_x.append(sx)
	sample_y.append(sy)
	sample_x.append(gx)
	sample_y.append(gy)
	return sample_x, sample_y


def main():
	n = 1
	for k in range(n):
		print(__file__ + " start!!")
		begin_time = time.time()
		# start and goal position
		sx = -40  # [m]
		sy = -40  # [m]
		gx = 40  # [m]
		gy = 40  # [m]
		robot_size = 2.0  # [m]
		ox, oy = [], []
		for i in range(-40, 20):
			ox.append(70)
			oy.append(i)
		for i in range(0, 70):
			ox.append(i)
			oy.append(-40)
		for i in range(40, 50):
			ox.append(0.0)
			oy.append(0 - i)
		for i in range(-50, 0):
			ox.append(i)
			oy.append(-50)
		for i in range(-50, 20):
			ox.append(-50)
			oy.append(i)
		for i in range(-70, -50):
			ox.append(i)
			oy.append(20)
		for i in range(20, 50):
			ox.append(-70)
			oy.append(i)
		for i in range(-70, 60):
			ox.append(i)
			oy.append(50)
		for i in range(20, 50):
			ox.append(60)
			oy.append(i)
		for i in range(60, 70):
			ox.append(i)
			oy.append(20)
		'''
		# Experiment 01 Start
		# people 1
		for i in range(-26, -24):
			ox.append(i)
			oy.append(16)
		for i in range(14, 16):
			ox.append(-24)
			oy.append(i)
		for i in range(-26, -24):
			ox.append(i)
			oy.append(14)
		for i in range(14, 16):
			ox.append(-26)
			oy.append(i)
		# people 2
		for i in range(14, 16):
			ox.append(i)
			oy.append(16)
		for i in range(14, 16):
			ox.append(16)
			oy.append(i)
		for i in range(14, 16):
			ox.append(i)
			oy.append(14)
		for i in range(14, 16):
			ox.append(14)
			oy.append(i)
		# people 3
		for i in range(-36, -34):
			ox.append(i)
			oy.append(-29)
		for i in range(-31, -29):
			ox.append(-34)
			oy.append(i)
		for i in range(-36, -34):
			ox.append(i)
			oy.append(-31)
		for i in range(-31, -29):
			ox.append(-36)
			oy.append(i)
		# people 4
		for i in range(19, 21):
			ox.append(i)
			oy.append(-29)
		for i in range(-31, -29):
			ox.append(21)
			oy.append(i)
		for i in range(19, 21):
			ox.append(i)
			oy.append(-31)
		for i in range(-31, -29):
			ox.append(19)
			oy.append(i)
		# setting 20x20 obstacle
		for i in range(-10, 10):
			ox.append(10)
			oy.append(i)
		for i in range(-10, 10):
			ox.append(i)
			oy.append(-10)
		for i in range(-10, 10):
			ox.append(-10)
			oy.append(i)
		for i in range(-10, 10):
			ox.append(i)
			oy.append(10)
		# Experiment 01 End
		

		# Experiment 02 Start
		for i in range(-10, 10):
			ox.append(10)
			oy.append(i)
		for i in range(-10, 10):
			ox.append(i)
			oy.append(-10)
		for i in range(-10, 10):
			ox.append(-10)
			oy.append(i)
		for i in range(-10, 10):
			ox.append(i)
			oy.append(10)

		for i in range(15,35):
			ox.append(45)
			oy.append(i)
		for i in range(25,45):
			ox.append(i)
			oy.append(15)
		for i in range(15,35):
			ox.append(25)
			oy.append(i)
		for i in range(25,45):
			ox.append(i)
			oy.append(35)

		for i in range(-35,-15):
			ox.append(45)
			oy.append(i)
		for i in range(25,45):
			ox.append(i)
			oy.append(-35)
		for i in range(-35,-15):
			ox.append(25)
			oy.append(i)
		for i in range(25,45):
			ox.append(i)
			oy.append(-15)

		for i in range(-35,-15):
			ox.append(-25)
			oy.append(i)
		for i in range(-45,-25):
			ox.append(i)
			oy.append(-35)
		for i in range(-35,-15):
			ox.append(-45)
			oy.append(i)
		for i in range(-45,-25):
			ox.append(i)
			oy.append(-15)

		for i in range(15,35):
			ox.append(-25)
			oy.append(i)
		for i in range(-45,-25):
			ox.append(i)
			oy.append(15)
		for i in range(15,35):
			ox.append(-45)
			oy.append(i)
		for i in range(-45,-25):
			ox.append(i)
			oy.append(35)

		# Experiment 02 End
		
		# Experiment 03 Start
		for i in range(-10, 10):
			ox.append(10)
			oy.append(i)
		for i in range(-10, 10):
			ox.append(i)
			oy.append(-10)
		for i in range(-10, 10):
			ox.append(-10)
			oy.append(i)
		for i in range(-10, 10):
			ox.append(i)
			oy.append(10)

		for i in range(15,35):
			ox.append(45)
			oy.append(i)
		for i in range(25,45):
			ox.append(i)
			oy.append(15)
		for i in range(15,35):
			ox.append(25)
			oy.append(i)
		for i in range(25,45):
			ox.append(i)
			oy.append(35)

		for i in range(-35,-15):
			ox.append(45)
			oy.append(i)
		for i in range(25,45):
			ox.append(i)
			oy.append(-35)
		for i in range(-35,-15):
			ox.append(25)
			oy.append(i)
		for i in range(25,45):
			ox.append(i)
			oy.append(-15)

		for i in range(-35,-15):
			ox.append(-25)
			oy.append(i)
		for i in range(-45,-25):
			ox.append(i)
			oy.append(-35)
		for i in range(-35,-15):
			ox.append(-45)
			oy.append(i)
		for i in range(-45,-25):
			ox.append(i)
			oy.append(-15)

		for i in range(15,35):
			ox.append(-25)
			oy.append(i)
		for i in range(-45,-25):
			ox.append(i)
			oy.append(15)
		for i in range(15,35):
			ox.append(-45)
			oy.append(i)
		for i in range(-45,-25):
			ox.append(i)
			oy.append(35)

		for i in range(-10,10):
			ox.append(45)
			oy.append(i)
		for i in range(25,45):
			ox.append(i)
			oy.append(-10)
		for i in range(-10,10):
			ox.append(25)
			oy.append(i)
		for i in range(25,45):
			ox.append(i)
			oy.append(10)

		for i in range(15,35):
			ox.append(10)
			oy.append(i)
		for i in range(-10,10):
			ox.append(i)
			oy.append(15)
		for i in range(15,35):
			ox.append(-10)
			oy.append(i)
		for i in range(-10,10):
			ox.append(i)
			oy.append(35)

		for i in range(-35,-15):
			ox.append(10)
			oy.append(i)
		for i in range(-10,10):
			ox.append(i)
			oy.append(-35)
		for i in range(-35,-15):
			ox.append(-10)
			oy.append(i)
		for i in range(-10,10):
			ox.append(i)
			oy.append(-15)

		for i in range(-10,10):
			ox.append(-25)
			oy.append(i)
		for i in range(-45,-25):
			ox.append(i)
			oy.append(-10)
		for i in range(-10,10):
			ox.append(-45)
			oy.append(i)
		for i in range(-45,-25):
			ox.append(i)
			oy.append(10)

		# Experiment 03 End
		
		'''
		# Experiment 04/05  Start

		d = 6
		l = 40
		for i in range(10 - l, 10):
			ox.append(70 - d)
			oy.append(i)
		for i in range(-50 + d, 70 - d):
			ox.append(i)
			oy.append(10 - l)
		for i in range(10 - l, 10):
			ox.append(-50 + d)
			oy.append(i)
		for i in range(-50 + d, 70 - d):
			ox.append(i)
			oy.append(10)

		# Experiment 04/05 End

		if show_animation:
			plt.plot(ox, oy, ".k")
			plt.plot(sx, sy, "^r")
			plt.plot(gx, gy, "^c")
			plt.grid(True)
			plt.axis("equal")
		rx, ry = prm_planning(sx, sy, gx, gy, ox, oy, robot_size)
		assert rx, 'Cannot found path'
		if show_animation:
			plt.plot(rx, ry, "-r")
			plt.pause(0.001)
			end_time = time.time()
			run_time = end_time - begin_time
			total_time.append(run_time)
			print('run time:', run_time)
			print('Result:',total_time)
			print('Length:', len(rx))
			plt.show()


if __name__ == '__main__':
	main()