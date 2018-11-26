"""
Author - Rajnish Tiwari
implementation of paper by Sertac Karaman
http://roboticsproceedings.org/rss06/p34.pdf

This is normal RRT* algorithm which will search a path 
between two given point start loaction and goal 
loaction while avoiding the given obstacles. It can be terminated
either once reached to goal location or when max iteration reached
depending on selection
"""

import math, sys, random, pygame
import envr

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

class RRTStar():
	def __init__(self, start, goal, obstacles, iter_limit = False, max_iter = MAX_ITER):
		self.start = Node(start)
		self.goalNode = Node(goal)
		self.obstacles = obstacles
		self.nodes = []
		self.nodes.append(self.start)
		self.iter_limit = iter_limit
		self.MAX_ITER = max_iter


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
		success = False
		for p in self.nodes:
			if self.dist(p.pos,point) <= self.dist(nearestPoint.pos,point):
				newPoint = self.steer(p.pos,point)
				if not self.checkObst(newPoint):
					nearestPoint = p
					success = True
		return success, nearestPoint

	def getRandompoint(self):
		"""
		Sample a random point in search space which is also in free space
		"""
		success = False
		while not success:
			point = self.getValidPoint()
			success, nearestPoint = self.getNearestpoint(point)
		return point, nearestPoint

	def getNearnodes(self, newNode):
		"""
		get all nodes within r distance from newNode
		where r = gamma*(log(no of nodes in tree)/no of nodes in tree)**
												(inv of no of dimension)
		"""
		nnodes = len(self.nodes)
		r = gamma*math.sqrt(math.log(nnodes)/nnodes)
		nearNodes = []
		for p in self.nodes:
			if self.dist(newNode.pos, p.pos) <= r:
				nearNodes.append(p)
		return nearNodes

	def chooseParent(self, newNode, nearNodes):
		"""
		select the parent node with lowest path cost from start node
		from all nodes within r distance
		"""
		if len(nearNodes) == 0:
			return newNode

		clist = []
		for p in nearNodes:
			clist.append(self.checkCost(p, newNode))

		minCost = min(clist)
		minNode = nearNodes[clist.index(minCost)]

		if minCost == float('inf'):
			return newNode

		newNode.cost = minCost
		newNode.parent = minNode
		return newNode


	def checkCost(self, p, newNode):
		"""
		get the path cost from node p to newNode
		if there is obstacle between nodes return
		infinite as path cost
		"""
		tmpNode = Node(p.pos)

		dy = newNode.pos[1] - p.pos[1]
		dx = newNode.pos[0] - p.pos[0]
		d = math.sqrt(dy**2 + dx**2)
		theta = math.atan2(dy, dx)
		for i in range(int(d/expansionDis)):
			x = tmpNode.pos[0] + expansionDis*math.cos(theta)
			y = tmpNode.pos[1] + expansionDis*math.sin(theta)
			tmpNode = Node((x,y))
			if self.checkObst(tmpNode.pos):
				return float('inf')

		return p.cost + d

	def updateTree(self, newNode, nearNodes):
		"""
		update the tree by reruiting the existing node which
		is less than r distance from newNode and if path cost
		through newNode is less than current cost
		"""
		for i in range(len(nearNodes)):
			cost = newNode.cost + self.dist(nearNodes[i].pos, newNode.pos)
			if nearNodes[i].cost > cost:
				if self.checkCost(nearNodes[i], newNode) != float('inf'):
					envr.drawPath(nearNodes[i].parent.pos, nearNodes[i].pos, envr.white)
					nearNodes[i].parent = newNode
					nearNodes[i].cost = cost
					envr.drawPath(newNode.pos, nearNodes[i].pos, envr.blue)


	def getNext(self):
		"""
		get the random generated node which is in free space
		update the tree by reruiting the existing node which
		is less than r distance from newNode and if path cost
		through newNode is less than current cost
		"""
		point, nearestPoint = self.getRandompoint()
		newNode = self.steer(nearestPoint.pos,point)
		newNode = Node(newNode, nearestPoint)
		nearNodes = self.getNearnodes(newNode)
		newNode = self.chooseParent(newNode, nearNodes)
		self.updateTree(newNode, nearNodes)
		return newNode

	def getNeargoal(self):
		"""
		find all nodes which is near to goal
		"""
		nearGoal = []
		for p in self.nodes:
			if self.dist(self.goalNode.pos, p.pos) <= envr.GOAL_RADIUS:
				nearGoal.append(p)
		return nearGoal

	def getBestnode(self):
		"""
		find the node near goal region with min path cost from start region
		"""
		nearGoal = self.getNeargoal()
		minCost = float('inf')
		minNode = None
		for p in nearGoal:
			if p.cost < minCost:
				minCost = p.cost
				minNode = p

		return minNode


	def run(self):
		"""
		Run the formulated RRT* algorithm when called
		'''''Algorithm''''''
		while iteration is less than maximum iteration
			newPoint <- sample rand in free space
			nearestPoint <- nearest node in tree
			newnode <- get point in direction of newPoint from nearestPoint if newPoint
					is more than the search step
			if newnode is in free space
				newnode.parent = minCost of all nearNode within distance r
								r = gamma*(log(no of nodes in tree)/no of nodes in tree)**inv(no of dimension)
				if cost < nearNode.cost + newnode.cost
					nearNode.parent = newnode	
				add newnode to tree
				if newnode is in goal region and not iteration limit
					return path
		if iteration limit
			find the path with lowest cost
			return path
		"""

		self.currentState = 'solving'
		self.count = 0

		while True:

			envr.screenUpdate()

			if self.currentState == 'solving':
				self.count += 1

				if self.count < self.MAX_ITER:
					newNode = self.getNext()
					self.nodes.append(newNode)
					envr.drawPath(newNode.parent.pos, newNode.pos, envr.cyan)

					if self.checkGoal(newNode) and not self.iter_limit:
						self.currentState = 'solved'
						self.goalNode.parent = newNode

				elif self.iter_limit:
					self.currentState = 'solved'
					bestNode = self.getBestnode()
					if bestNode == None:
						print("Maximum Number of Iteration Reached")

					self.goalNode.parent = bestNode				

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

