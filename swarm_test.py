#!/usr/bin/env python2

from math import cos, cosh, pi, sin
import time
import numpy as np

from numpy.core.records import array
import rospy

import time
from threading import Thread

# import queue

from swarm_controller import *
# import xlsxwriter 


def main():

	rospy.init_node('tello_swarm_test_node', anonymous=True)

	num_of_drones = 3

	objective_positions = [
		[2.5,	0,	1],
		[0.5, 0.0, 1.5],
		[0, 1, 1]
	]

	objective_paths = [
		[[0,	0,	1], [0, 0, 1]],
		[[0.5, 0.0, 1.5], [1.5, 0.0, 1.5]],
		[[0, 1, 1], [1, 1, 1]]
	]
	
	tags = [
		"tello0",
		"tello1",
		"tello2"
	]

	swarm = TelloSwarm(num_of_drones, objective_positions, tags, objective_paths)

	# design a trajectory
	pos_traj = []
	ini_obj_position= swarm.get_agent_pos(0).copy()

	swarm.parallel_takeoff()
	#swarm.agent_takeoff(0)
	# swarm.parallel_takeoff_positioning()
	time.sleep(5)
	#swarm.parallel_takeoff()
	
	# swarm.run() # run the drone to move
	swarm.run_rlInference()

	# swarm.run_agent()
	swarm.parallel_land()
	rospy.loginfo("Landing swarm.")


if __name__ == '__main__':
	main()
