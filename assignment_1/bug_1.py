from helper import Point2D, PointsToLine2D, Polygon2D
from bug_base import parseInputs

class Bug1AlgoClass():

	def __init__(self, start, goal, step_size, obstaclesList, verbose = False):

		self.startPoint = Point2D(start[0], start[1])
		self.goalPoint = Point2D(goal[0], goal[1])

		self.step_size = step_size
		self.obstaclesList = obstaclesList

		self.path = []
		self.currentPosition = self.startPoint

		self.closestPolygon = None
		self.closestDistance = float('inf')

		self.vector = (None, None)

		self.closestToGoalDist = float('inf')
		self.closestToGoalPoint = self.startPoint

		self.verbose = verbose


	def findClosestPolygon(self):

		self.closestDistance = float('inf')

		for obstacle in self.obstaclesList:

			polygon = Polygon2D(len(obstacle), obstacle)
			distance, _, _ = polygon.computeDistancePointToPolygon(self.currentPosition)

			if distance < self.closestDistance:
				self.closestDistance = distance
				self.closestPolygon = polygon


	def moveAndUpdatePath(self):

		self.currentPosition.move(Point2D(self.step_size*self.vector[0], self.step_size*self.vector[1]))

		self.path.append((self.currentPosition._x, self.currentPosition._y))

		# if self.verbose:
		# 	print(self.path[-1])


	def circumNavigation(self):

		n = self.closestPolygon._n
		V = self.closestPolygon._V

		centroid = Point2D(sum([V[x][0] for x in range(n)])/n, sum([V[x][1] for x in range(n)])/n)

		initVector = (self.currentPosition-centroid)/self.currentPosition.euclideanDistance(centroid)

		circumnavProximityLimit = int(1/self.step_size)

		circumnavOrigin = Point2D(self.currentPosition._x, self.currentPosition._y)

		self.closestToGoalDist = self.goalPoint.euclideanDistance(self.currentPosition)
		self.closestToGoalPoint = self.currentPosition

		self.vector = self.closestPolygon.computeTangentVectorToPolygon(self.currentPosition)

		self.moveAndUpdatePath()

		circumNavLocalCnt = 1

		# while circumNavLocalCnt < circumnavProximityLimit or circumnavOrigin.euclideanDistance(self.currentPosition)>= self.step_size:
		while circumNavLocalCnt < circumnavProximityLimit or (initVector*newVector).coordinateSum()<0.9:

			localGoalDist = self.goalPoint.euclideanDistance(self.currentPosition)

			if localGoalDist < self.closestToGoalDist:

				self.closestToGoalDist = localGoalDist
				self.closestToGoalPoint = Point2D(self.currentPosition._x, self.currentPosition._y)

			self.vector = self.closestPolygon.computeTangentVectorToPolygon(self.currentPosition)

			self.moveAndUpdatePath()

			newVector = (self.currentPosition-centroid)/self.currentPosition.euclideanDistance(centroid)

			circumNavLocalCnt += 1

			# if circumNavLocalCnt > 300:
			# 	break


	def bug1Algorithm(self):

		circumNavCnt = 0

		while self.goalPoint.euclideanDistance(self.currentPosition) > self.step_size:

			self.findClosestPolygon()

			if self.closestDistance < 2*self.step_size:

				self.circumNavigation()
				
				# move to point closest to goal
				goalLine = PointsToLine2D(self.closestToGoalPoint, self.goalPoint)
				while goalLine.computeDistancePointToSegment(self.currentPosition) > 2*self.step_size:

					self.vector = self.closestPolygon.computeTangentVectorToPolygon(self.currentPosition)
					
					self.moveAndUpdatePath()

				# check step possibility
				L = PointsToLine2D(self.currentPosition, self.goalPoint)

				self.vector = (L.b, -L.a)

				self.moveAndUpdatePath()

				if self.closestPolygon.computeDistancePointToPolygon(self.currentPosition)[0] < self.step_size:
					return "Failure: No possible path to reach goal point", self.path

			L = PointsToLine2D(self.currentPosition, self.goalPoint)
			
			self.vector = (L.b, -L.a)

			self.moveAndUpdatePath()

		self.path.append((self.goalPoint._x, self.goalPoint._y))

		return "Success", self.path

def plotAndSavePath(path, start, goal, goalDistanceLog):

	np.savetxt('data/output_bug1.txt', path)

	plt.plot(goalDistanceLog)
	plt.savefig('data/goalDistFromBug.png')

	plt.scatter(*np.array(path).T, s=4)
	plt.plot(start[0], start[1], 'ro', markersize=10)
	plt.plot(goal[0], goal[1],'go', markersize=10)

	for i in range(len(obstaclesList)):
		t1 = plt.Polygon(obstaclesList[i], color = 'red', fill = False)
		plt.gca().add_patch(t1)

	plt.savefig('data/path.png')

def main():
	start, goal, step_size, obstaclesList = parseInputs('input.txt')
	algo = Bug1AlgoClass(start, goal, step_size, obstaclesList, 'False')
	status, path = algo.bug1Algorithm()
	print(status)

	import numpy as np
	np.savetxt('data/output_bug1.txt', path)
	from matplotlib import pyplot as plt
	plt.scatter(*np.array(path).T, s=4)
	plt.plot(start[0], start[1], 'ro', markersize=10)
	plt.plot(goal[0], goal[1],'go', markersize=10)
	for i in range(len(obstaclesList)):
		t1 = plt.Polygon(obstaclesList[i], color = 'red', fill = False)
		plt.gca().add_patch(t1)
	plt.show()
	# print('\n'.join([str(x) for x in path]))

if __name__ == '__main__':
	main()