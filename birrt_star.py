"""
Author - Rajnish Tiwari

This is normal Bi-direction RRT* algorithm which will search a path 
between two given point start loaction and goal 
loaction while avoiding the given obstacles. It can be terminated
either once reached to goal location or when max iteration reached
depending on selection
"""
import math, sys, random, pygame
import envr

from rrt_star import RRTStar

delta = 10.0
MIN_DISTANCE_TO_ADD = 1.0
MAX_ITER = 5000

gamma = 300
expansionDis = 4

screen = envr.screen


class Node():
    def __init__(self, pos, parent = None):
        self.pos = pos
        self.parent = parent
        self.cost = 0



class BIRRTStar():
	def __init__(self, start, goal, obstacles, iter_limit = False):
		"""
		initialize the Bi-directional RRT* with RRT*  from start
		and RRT* from goal
		"""
		self.start = Node(start)
		self.goalNode = Node(goal)
		self.obstacles = obstacles
		self.iter_limit = iter_limit
		self.RRT_from_start = RRTStar(start, goal, obstacles)
		self.RRT_from_goal = RRTStar(goal, start, obstacles)

	def dist(self, p1, p2):
		"""
		get the distance between point p1 and point p2
		"""
		return math.sqrt((p1[0]-p2[0])**2 +(p1[1]-p2[1])**2)

	def getPath(self, node_from_start, node_from_goal):
		"""
		connect node of RRT* from start and RRT* from goal to 
		make path from start region to goal region
		"""
		currNode = node_from_goal
		parentNode = node_from_start
		while currNode != None:
			tmpNode = currNode.parent
			currNode.parent = parentNode
			parentNode = currNode
			currNode = tmpNode

	def getBestnodes(self):
		"""
		Return the pair of nodes from RRT* from start and RRT* from goal
		which have the lowest path cost from start to goal region
		"""
		best_from_start = None
		best_from_goal = None
		minCost = float('inf')

		for s in self.RRT_from_start.nodes:
			for g in self.RRT_from_goal.nodes:
				if self.dist(s.pos, g.pos) <= delta and (s.cost+ g.cost) < minCost:
					best_from_start = s 
					best_from_goal = g
					minCost = s.cost+ g.cost

		return best_from_start, best_from_goal


	def run(self):
		"""
		Run the formulated Bi-directional RRT* algorithm when called
		'''''Algorithm''''''
		while iteration is less than maximum iteration
			if iteration even
				run RRT* from start location
				newNode_start = RRT* from start
			if iteration odd
				run RRT* from goal location
				newNode_goal = RRT* from goal
			if newNode_start is less then search step from 
				any node in RRT* from goal and not iteration limit
				return path
			elif newNode_goal is less then search step from 
				any node in RRT* from start and not iteration limit
				return path
		if iteration limit
			find the path with lowest cost by connecting the RRT* from start
			and RRT* from goal
			return path
		"""
		self.currentState = 'solving'
		self.count = 0

		success_to_start = False
		success_to_goal = False

		while True:

			envr.screenUpdate()

			if self.currentState == 'solving':
				self.count += 1

				if self.count < MAX_ITER:
					if self.count%2:
						newNode_from_start = self.RRT_from_start.getNext()
						self.RRT_from_start.nodes.append(newNode_from_start)
						envr.drawPath(newNode_from_start.parent.pos, newNode_from_start.pos, envr.cyan)
						success_to_goal, nearestPoint_in_goal = self.RRT_from_goal.getNearestpoint(newNode_from_start.pos)
					else:
						newNode_from_goal = self.RRT_from_goal.getNext()
						self.RRT_from_goal.nodes.append(newNode_from_goal)
						envr.drawPath(newNode_from_goal.parent.pos, newNode_from_goal.pos, envr.cyan)
						success_to_start, nearestPoint_in_start = self.RRT_from_start.getNearestpoint(newNode_from_goal.pos)

					if not self.iter_limit:

						if success_to_goal and self.dist(newNode_from_start.pos, nearestPoint_in_goal.pos) <= delta:
							self.currentState = 'solved'
							self.path = self.getPath(newNode_from_start, nearestPoint_in_goal)
	
						elif success_to_start and self.dist(nearestPoint_in_start.pos, newNode_from_goal.pos) <= delta:
							self.currentState = 'solved'
							self.path = self.getPath(nearestPoint_in_start, newNode_from_goal)
				
				elif self.iter_limit:
					best_from_start, best_from_goal = self.getBestnodes()
					if best_from_start == None or best_from_goal == None:
						print("Maximum Number of Iteration Reached")
						return

					self.currentState = 'solved'
					self.path = self.getPath(best_from_start, best_from_goal)

				else:
					print("Maximum Number of Iteration Reached")
					return

			elif self.currentState == 'solved':
				currNode = self.RRT_from_goal.start

				while currNode.parent != None:
					envr.drawPath(currNode.pos,currNode.parent.pos, envr.red, 6)
					currNode = currNode.parent

				pygame.display.set_caption('Found the path')

			for e in pygame.event.get():
				if e.type == pygame.QUIT or (e.type == pygame.KEYUP and e.key == pygame.K_ESCAPE):
					sys.exit("Initiated Exit")

