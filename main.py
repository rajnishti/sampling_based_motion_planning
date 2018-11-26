"""
Topic - Motion Planning
Author - Rajnish Tiwari

This is main file which will import motion planning algorithms
like RRT, RRT* and Bi-Directional RRT*
This function will first call a screen of all these motion planning
algorithms then it will create few default obstacles. After this
it will ask to select starting point and goal point which can 
be selected by clicking the mouse on screen 
"""

import pygame

import envr
from rrt import RRT
from rrt_star import RRTStar
from birrt_star import BIRRTStar


def main():
	s = envr.initialScreen()

	if s != 'RRT':
		iter_limit = envr.drawIterlimit()

	obstacles = envr.getObstacle(4)
	envr.drawObstacles(obstacles)
	points = envr.getSearchPoints(obstacles)

	if s == 'RRT':
		planner = RRT(points[0], points[1], obstacles)
		pygame.display.set_caption('RRT searching for path')
	elif s == 'RRT*':
		planner = RRTStar(points[0], points[1], obstacles, iter_limit)
		pygame.display.set_caption('RRT* searching for path')
	else:
		planner = BIRRTStar(points[0], points[1], obstacles, iter_limit)
		pygame.display.set_caption('Bi-RRT* searching for path')

	planner.run()


if __name__ == '__main__':

	main()