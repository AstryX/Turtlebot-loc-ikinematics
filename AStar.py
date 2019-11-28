#Written by Robertas Dereskevicius 2019/11 University of Edinburgh
#Basic A* algorithm for grid path planning
import numpy as np
import random

#Basically a tile object which behaves like a node in a linked list with extra data
class CustomTile:
	def __init__(self, _worldPos=[0,0], _cellPos=[0,0], _isObstacle=False):
		self.worldPos = _worldPos
		self.cellPos = _cellPos
		self.isObstacle = _isObstacle
		self.prevTile = None
		self.G = 0
		self.H = 0
		
	def GetPrev(self):
		return self.prevTile
		
	def SetPrev(self,_prev):
		self.prevTile = _prev
		
	def SetG(self,_g):
		self.G = _g
		
	def SetH(self,_h):
		self.H = _h
		
	def GetG(self):
		return self.G
		
	def GetH(self):
		return self.H
		
	def GetF(self):
		return self.G + self.H
		
	def GetIsObstacle(self):
		return self.isObstacle
		
	def GetCellPos(self):
		return self.cellPos
	
#Behaves like a linked list manager and stores world occupancy grid(map) and the dictionary of coords	
class BotPathData:
	def __init__(self, _map):
		self.map = _map
		self.tileDictionary = {}

		for n in range(len(_map)):
			for p in range(len(_map[n])):
				localPlace = [n, p]
				key = str(n) + "," + str(p)
				if key not in self.tileDictionary:
					self.tileDictionary[key] = CustomTile(localPlace, localPlace, False if self.map[n][p] == 0 else True)
		
	#Semi-legacy, the algo was initially designed to work with grid + real world positions
	#from which tile of the grid was located
	def TileFromCellPos(self, _x, _y):
		key = str(_x) + "," + str(_y)
		if key in self.tileDictionary:
			return self.tileDictionary[key]

		return None;

	#Getting neighbours of front, back, left, right of a tile
	def GetNeighbours(self, _cur):
		neighbours = []

		xShift = _cur.GetCellPos()[0] + 1
		yShift = _cur.GetCellPos()[1]
		key = str(xShift) + "," + str(yShift)
		if key in self.tileDictionary:
			neighbours.append(self.tileDictionary[key])

		xShift = _cur.GetCellPos()[0]
		yShift = _cur.GetCellPos()[1] + 1
		key = str(xShift) + "," + str(yShift)
		if key in self.tileDictionary:
			neighbours.append(self.tileDictionary[key])

		xShift = _cur.GetCellPos()[0] - 1
		yShift = _cur.GetCellPos()[1]
		key = str(xShift) + "," + str(yShift)
		if key in self.tileDictionary:
			neighbours.append(self.tileDictionary[key])

		xShift = _cur.GetCellPos()[0]
		yShift = _cur.GetCellPos()[1] - 1
		key = str(xShift) + "," + str(yShift)
		if key in self.tileDictionary:
			neighbours.append(self.tileDictionary[key])

		return neighbours

#Main class that runs the A* algo and retrieves a path
class Pathfinding:
	def __init__(self, _map):
		self.botData = BotPathData(_map)
		self.pathToDestination = []
		
	
	def GetPathToDestination(self):
		return self.pathToDestination
	
	def CalculateFinalPath(self, startCoords, destCoords):
		discoveredTiles = set()
		discardedTiles = set()
		current = self.botData.TileFromCellPos(startCoords[0], startCoords[1])
		dest = self.botData.TileFromCellPos(destCoords[0], destCoords[1])
		
		discoveredTiles.add(current)

		while len(discoveredTiles) > 0:
			presentTile = CustomTile()
			init = True
			for it in discoveredTiles:
				if init == True:
					presentTile = it
					init = False
				else:
					itTile = it
					if itTile.GetF() < presentTile.GetF() or itTile.GetF() == presentTile.GetF() and itTile.GetH() < presentTile.GetH():
						presentTile = itTile
			discoveredTiles.remove(presentTile)
			discardedTiles.add(presentTile)
			
			if presentTile.GetCellPos()[0] == dest.GetCellPos()[0] and presentTile.GetCellPos()[1] == dest.GetCellPos()[1]:
				self.GetSequence(current, dest)
			
			for curNeighbour in self.botData.GetNeighbours(presentTile):
				if curNeighbour.GetIsObstacle() == True or curNeighbour in discardedTiles:
					continue
					
				curCellPos = current.GetCellPos()
				neighCellPos = curNeighbour.GetCellPos()
				
				cost = current.GetG() + abs(curCellPos[0] - neighCellPos[0]) + abs(curCellPos[1] - neighCellPos[1])
				if cost < curNeighbour.GetG() or curNeighbour not in discoveredTiles:
					destCellPos = dest.GetCellPos()
					curNeighbour.SetG(cost)
					curNeighbour.SetH(abs(neighCellPos[0] - destCellPos[0]) + abs(neighCellPos[1] - destCellPos[1]))
					
					curNeighbour.SetPrev(presentTile)
					
					if curNeighbour not in discoveredTiles:
						discoveredTiles.add(curNeighbour)
		
	def GetSequence(self, _cur, _dest):
		path = []
		current = _dest
		while (current != _cur):
			path.append(current)
			current = current.GetPrev()
			
		path.append(_cur)
		path = list(reversed(path))
		self.pathToDestination = path 
		
def computeGridPathPOI(map, startPos, endPos):
	simulation = Pathfinding(map)
	simulation.CalculateFinalPath(startPos, endPos)
	listPOI = []
	fullPath = simulation.GetPathToDestination()

	prevX = []
	prevY = []
	prevDiffX = 0
	prevDiffY = 0
	i = 0
	for tile in simulation.GetPathToDestination():
		if i == 0:
			prevX = tile.GetCellPos()[0]
			prevY = tile.GetCellPos()[1]
		elif i == 1:
			prevDiffX = tile.GetCellPos()[0] - prevX
			prevDiffY = tile.GetCellPos()[1] - prevY
			prevX = tile.GetCellPos()[0]
			prevY = tile.GetCellPos()[1]
		else:
			curX = tile.GetCellPos()[0]
			curY = tile.GetCellPos()[1]

			diffX = curX - prevX
			diffY = curY - prevY

			if diffX == prevDiffX and diffY == prevDiffY:
				prevDiffX = diffX
				prevDiffY = diffY
				prevX = curX
				prevY = curY
			else:
				listPOI.append([prevX, prevY])
				prevDiffX = diffX
				prevDiffY = diffY
				prevX = curX
				prevY = curY
		
		i += 1
	listPOI.append([prevX, prevY])

	return listPOI

'''	map = np.zeros((50,50), dtype=int)
maxMapX = 50
maxMapY = 50
print("Simulated Map:")
for i in range(maxMapX):
	out = ""
	for j in range(maxMapY):
		v1 = random.randint(1, 5) % 5
		if v1 == 0:
			map[i][j] = 1
		out = out + ("|" + str(map[i][j]))
	print(out)		
	
simulation = Pathfinding(map)
simulation.CalculateFinalPath([1,1], [45,45])
for tile in simulation.GetPathToDestination():
	print("(" + str(tile.GetCellPos()[0]) + "," + str(tile.GetCellPos()[1]) + ")")

listPOI = computeGridPathPOI(map, [1,1], [45,45])
print(listPOI)
print("Path processing completed!")'''