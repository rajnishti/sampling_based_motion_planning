"""
Author - Rajnish Tiwari

This part of code will basically create the pygame screen and 
generate obstacles for motion planning algorithms.
Size of screen can be changed by changing the XDIM and YDIM
Start point and goal point radius can be changed by 
changing the GOAL_RADIUS
"""


import math, sys, random, pygame
pygame.init()


XDIM = 950   	# width of pygame screen
YDIM = 600		# height  of pygame screen
windowSize = [XDIM, YDIM]
GOAL_RADIUS = 10 # radius of starting and goal point

fpsClock = pygame.time.Clock()
screen = pygame.display.set_mode(windowSize)

white = 255, 255, 255
black = 0, 0, 0
red = 255, 0, 0
blue = 0, 0, 255
green = 0, 255, 0
cyan = 0,180,105


class line():
	"""
	this class will be used to include line type obstacles
	in pygame environment.
	"""
	def __init__(self, color,pointi, pointg, width = 5):		
		self.color = color
		self.pointi = pointi
		self.pointg = pointg
		self.width = width

	def draw(self,outline = None):
		"""
		draw the obstacle on screen
		"""
		pygame.draw.line(screen, self.color, self.pointi, self.pointg, self.width)

	def isCollide(self, pos):
		"""
		return True if given position lies inside the obstacle
		"""
		if pos[0] > self.pointi[0] and pos[0] < self.pointg[0]:
			if pos[1] > self.pointi[1] and pos[1] < self.pointg[1]:
				return True

		return False



class circle():
	"""
	this class will be used to include circle type obstacles
	in pygame environment.
	"""
	def __init__(self,color, x, y, r):
		self.color = color
		self.x = x 
		self.y = y 
		self.r = r 

	def draw(self,outline = None):
		"""
		draw the obstacle on screen
		"""
		pygame.draw.circle(screen, self.color, (self.x, self.y), self.r)

	def isCollide(self, pos):
		"""
		return True if given position lies inside the obstacle
		"""
		if math.sqrt((pos[0] - self.x)**2 + (pos[1] - self.y)**2) <= self.r:
			return True

		return False

class rectangle():
	"""
	this class will be used to include rectangle type obstacles
	in pygame environment.
	"""
	def __init__(self, color, x, y, width, height):
		self.color = color
		self.x = x
		self.y = y 
		self.width = width
		self.height  = height 

	def draw(self,outline = None):
		"""
		draw the obstacle on screen
		"""		
		pygame.draw.rect(screen, self.color, (self.x, self.y, self.width, self.height), 0)

	def isCollide(self, pos):
		"""
		return True if given position lies inside the obstacle
		"""
		if pos[0] > self.x and pos[0] < self.x + self.width:
			if pos[1] > self.y and pos[1] < self.y + self.height:
				return True

		return False


class button():
	"""
	this class create button with given text on pygame screen
	"""
	def __init__(self, color, x, y, text='', width = 120,height = 30):
		self.color = color
		self.x = x
		self.y = y
		self.width = width
		self.height = height
		self.text = text

	def draw(self,outline = None):
		"""
		Draw the button with given text width and height
		"""
		if outline:
			pygame.draw.rect(screen, outline, (self.x-2,self.y-2,self.width+4,self.height+4),0)
			
		pygame.draw.rect(screen, self.color, (self.x,self.y,self.width,self.height),0)

		if self.text != '':
			font = pygame.font.SysFont('comicsans', self.height - 2)
			text = font.render(self.text, 1, (0,0,0))
			screen.blit(text, (self.x + (self.width/2 - text.get_width()/2), self.y + (self.height/2 - text.get_height()/2)))

	def isOver(self, pos):
		"""
		return True if motion position is on button
		"""
		if pos[0] > self.x and pos[0] < self.x + self.width:
			if pos[1] > self.y and pos[1] < self.y + self.height:
				return True

		return False

def getObstacle(n):
	"""
	Return the n number of obstacles class randomly from 
	obstacle type defined in obs_type
	"""
	obs_type = {1: 'Rect', 2: 'Circle'}

	obstacles = []

	obs = random.randint(1, 2)
	for i in range(n):
		x = random.randint(10,XDIM - 200)
		y = random.randint(10, YDIM - 150)

		if obs_type[obs] == obs_type[1]:
			obstacles.append(rectangle(black, x, y, 180, 120))

		elif obs_type[obs] == obs_type[2]:
			obstacles.append(circle(black, x+100, y+100, 75))

		obs = random.randint(1,2)

	return obstacles


def initialScreen():
	"""
	draw the pygame screen with motion planning algorithm button
	wait for user to select the button
	"""

	algo = {1: 'RRT', 2: 'RRT*', 3: 'BI_RRT*'}

	algoButton = []
	i = 0
	for a in algo:
		algoButton.append(button(green, XDIM/2 - 250, i*150 + 10, algo[a], 500, 100))
		i +=1

	screen.fill(white)

	while True:

		redraw(algoButton, True)
		pygame.display.update()
		fpsClock.tick(10000)

		for e in pygame.event.get():
			if e.type == pygame.QUIT or (e.type == pygame.KEYUP and e.key == pygame.K_ESCAPE):
				sys.exit("Initiated Exit")

			elif e.type == pygame.MOUSEBUTTONDOWN:
					
				for b in algoButton:
					if b.isOver(e.pos) and b.text == 'RRT':
						print 'RRT selected'
						return b.text

					elif b.isOver(e.pos) and b.text == 'RRT*':
						print 'RRT* selected'
						return b.text
							
					elif b.isOver(e.pos) and b.text == 'BI_RRT*':
						print 'BI_RRT* selected'
						return b.text


			if e.type == pygame.MOUSEMOTION:
					for b in algoButton:
						if b.isOver(e.pos):
							b.color = red
			
						else:
							b.color = green



def drawObstacles(obstacles):
	"""
	draw set of obstacles on screen
	"""
	screen.fill(white)
	redraw(obstacles)
	pygame.display.update()


def drawIterlimit():
	"""
	get the screen with option goal reach and iteration for RRT* and BI_RRT* 
	""" 
	options = {1: 'Goal Reach', 2: 'Iter Limit'}

	optionButton = []
	i = 0
	for p in options:
		optionButton.append(button(green, XDIM/2 - 250, i*150 + 10, options[p], 500, 100))
		i +=1

	screen.fill(white)

	while True:

		redraw(optionButton, True)
		pygame.display.update()
		fpsClock.tick(10000)

		for e in pygame.event.get():
			if e.type == pygame.QUIT or (e.type == pygame.KEYUP and e.key == pygame.K_ESCAPE):
				sys.exit("Initiated Exit")

			elif e.type == pygame.MOUSEBUTTONDOWN:
					
				for b in optionButton:
					if b.isOver(e.pos) and b.text == 'Goal Reach':
						print 'Termination by goal reach selected'
						return False

					elif b.isOver(e.pos) and b.text == 'Iter Limit':
						print 'Termination by Iteration limit selected'
						return True


			if e.type == pygame.MOUSEMOTION:
					for b in optionButton:
						if b.isOver(e.pos):
							b.color = red	
						else:
							b.color = green


def redraw(arg, outline = None):
	"""
	draw the sets of shapes givens in arg
	"""
	for b in arg:
		b.draw(outline)


def getSearchPoints(obstacles):
	""" get the initial point and goal point from used
	after clicking on the pygame screen
	"""

	initPoseSet = False
	while True:
		for e in pygame.event.get():
			if e.type == pygame.QUIT or (e.type == pygame.KEYUP and e.key == pygame.K_ESCAPE):
				sys.exit("Initiated Exit")

			if not initPoseSet:
					pygame.display.set_caption('Select the Starting Point')
					pygame.display.update()

			if e.type == pygame.MOUSEBUTTONDOWN:
				if initPoseSet == False:
						
					if checkObst(e.pos, obstacles) == False:
						print('initiale point set: '+str(e.pos))
						initialPoint = e.pos
						initPoseSet = True
						pygame.draw.circle(screen, red, (initialPoint[0], initialPoint[1]), GOAL_RADIUS)
						pygame.display.set_caption('Select the Goal Point')
						pygame.display.update()

				else:
					if checkObst(e.pos, obstacles) == False:
						print('goal point set: '+str(e.pos))
						goalPoint = e.pos
						pygame.draw.circle(screen, green, (goalPoint[0], goalPoint[1]), GOAL_RADIUS)
						return initialPoint, goalPoint
	


def checkObst(pos, obstacles):
	"""
	return True if given position lies in the sets
	of obstacles
	"""
	for obs in obstacles:
		if obs.isCollide(pos) == True:
			return True
	return False


def drawPath(pos1, pos2, color, width = 1):
	"""
	draw the line between position1 to position2
	"""
	pygame.draw.line(screen,color,pos1,pos2, width)

def screenUpdate():
	"""
	update the screen
	"""
	pygame.display.update()
	fpsClock.tick(10000)

	