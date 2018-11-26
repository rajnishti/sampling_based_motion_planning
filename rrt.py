"""
Author - Rajnish Tiwari

This is normal RRT algorithm which will search a path 
between two given point start loaction and goal 
loaction while avoiding the given obstacles.
""" 

import math, sys, random, pygame
import envr

delta = 10.0
MIN_DISTANCE_TO_ADD = 1.0
MAX_ITER = 5000

screen = envr.screen


class Node():
    def __init__(self, pos, parent = None):
        self.pos = pos
        self.parent = parent

class RRT():
	"""
	Create the RRT class to search the path between start and goal
	while avoiding the obstacles.
	"""
	def __init__(self, start, goal, obstacles):
		self.start = Node(start)
		self.goalNode = Node(goal)
		self.obstacles = obstacles
		self.nodes = []
		self.nodes.append(self.start)


	def checkObst(self, pos):
		"""
		Check if given position is in free space
		"""
		for obs in self.obstacles:
		    if obs.isCollide(pos) == True:
		        return True
		return False

	def getValidPoint(self, XDIM = envr.XDIM, YDIM = envr.YDIM):
		"""
		Generate a random point which is in free sapce
		"""
		while True:
		    p = (random.random()*XDIM, random.random()*YDIM)
		    if not self.checkObst(p):
		        return p

	def dist(self, p1, p2):
		"""
		Distance between two points
		"""
		return math.sqrt((p1[0]-p2[0])**2 +(p1[1]-p2[1])**2)


	def checkGoal(self, node):
		"""
		Check if point reached to the goal region
		"""
		if (self.dist(node.pos, self.goalNode.pos) <= envr.GOAL_RADIUS):
			return True

		return False

	def steer(self, p1, p2):
		"""
		Return a point which is in direction of p2 but
		closer to p1 than p2 if p2 is more then search step
		"""  
		if self.dist(p1,p2) < delta:
			return p2
		else:
			theta = math.atan2(p2[1]-p1[1],p2[0]-p1[0])
			return p1[0] + delta*math.cos(theta), p1[1] + delta*math.sin(theta)

	def getNearestpoint(self, point):
		"""
		Return the a node in tree which is nearest to given point
		"""
		nearestPoint = self.nodes[0]
		sucess = False
		for p in self.nodes:
			if self.dist(p.pos,point) <= self.dist(nearestPoint.pos,point):
				newPoint = self.steer(p.pos,point)
				if not self.checkObst(newPoint):
					nearestPoint = p
					sucess = True
		return sucess, nearestPoint

	def getRandompoint(self):
		"""
		Sample a random point in search space which is also in free space
		"""
		sucess = False
		while not sucess:
			point = self.getValidPoint()
			sucess, nearestPoint = self.getNearestpoint(point)
		return point, nearestPoint

	def getNext(self):
		"""
		get the random generated node which is in free space
		"""
		point, parentNode = self.getRandompoint()
		newnode = self.steer(parentNode.pos,point)
		return Node(newnode, parentNode)

	def run(self):
		"""
		Run the formulated RRT algorithm when called
		'''''Algorithm''''''
		while iteration is less than maximum iteration
			newPoint <- sample rand in free space
			nearestPoint <- nearest node in tree
			newnode <- get point in direction of newPoint from nearestPoint if newPoint
					is more than the search step
			newnode.parent = nearestPoint
			if newnode is  in search space
				add newnode to tree
				if newnode is in goal region
					return path
		"""

		self.currentState = 'solving'
		self.count = 0

		while True:

			envr.screenUpdate()

			if self.currentState == 'solving':
				self.count += 1

				if self.count < MAX_ITER:
					newNode = self.getNext()
					self.nodes.append(newNode)
					envr.drawPath(newNode.parent.pos, newNode.pos, envr.cyan)

					if self.checkGoal(newNode):
						self.currentState = 'solved'
						self.goalNode.parent = newNode

				else:
					print("Maximum Number of Iteration Reached")
					return

			elif self.currentState == 'solved':
				currNode = self.goalNode
				pygame.display.set_caption('Found the path')

				while currNode.parent != None:
					envr.drawPath(currNode.pos,currNode.parent.pos, envr.red, 6)
					currNode = currNode.parent

			for e in pygame.event.get():
				if e.type == pygame.QUIT or (e.type == pygame.KEYUP and e.key == pygame.K_ESCAPE):
					sys.exit("Initiated Exit")

