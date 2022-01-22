#!/usr/bin/env python3

from helper import Point2D, PointsToLine2D, Polygon2D
from bug_base import parseInputs

def findClosestPolygon(obstaclesList, currentPosition):

	# find polygon closest to currentPosition
	closestPolygon = None
	closestDistance = float('inf')

	for obstacle in obstaclesList:
		polygon = Polygon2D(len(obstacle), obstacle)
		distance, _, _ = polygon.computeDistancePointToPolygon(currentPosition)
		if distance < closestDistance:
			closestDistance = distance
			closestPolygon = polygon	
	return closestPolygon, closestDistance

def updatePath(path, point, verbose = True):

	path.append((point._x, point._y))

	if verbose:
		print(path[-1])

	return path

def circumNavigation(closestPolygon, closestDistance, currentPosition, goalPoint, path, step_size):

	circumnavProximityLimit = 50

	circumnavOrigin = Point2D(currentPosition._x, currentPosition._y)

	closestToGoalDist = goalPoint.euclideanDistance(currentPosition)
	closestToGoalPoint = currentPosition

	vector = closestPolygon.computeTangentVectorToPolygon(currentPosition)
	currentPosition.move(Point2D(step_size*vector[0], step_size*vector[1]))

	path = updatePath(path, currentPosition)

	circumNavLocalCnt = 1

	while circumNavLocalCnt < circumnavProximityLimit or circumnavOrigin.euclideanDistance(currentPosition)>=step_size:

		localGoalDist = goalPoint.euclideanDistance(currentPosition)

		if localGoalDist < closestToGoalDist:

			closestToGoalDist = localGoalDist
			closestToGoalPoint = Point2D(currentPosition._x, currentPosition._y)

		vector = closestPolygon.computeTangentVectorToPolygon(currentPosition)

		currentPosition.move(Point2D(step_size*vector[0], step_size*vector[1]))

		path = updatePath(path, currentPosition)

		circumNavLocalCnt += 1

	return currentPosition, path, closestToGoalPoint, closestToGoalDist



def bug1Algorithm(start, goal, step_size, obstaclesList):

	startPoint = Point2D(start[0], start[1])
	goalPoint = Point2D(goal[0], goal[1])

	currentPosition = startPoint

	path = updatePath([], currentPosition)

	while goalPoint.euclideanDistance(currentPosition) > step_size:

		closestPolygon, closestDistance = findClosestPolygon(obstaclesList, currentPosition)

		if closestDistance < 1.5*step_size:
			
			currentPosition, path, closestToGoalPoint, closestToGoalDist = circumNavigation(closestPolygon, closestDistance, currentPosition, goalPoint, path, step_size)
			
			# move to point closest to goal
			while closestToGoalPoint.euclideanDistance(currentPosition) > step_size:
				vector = closestPolygon.computeTangentVectorToPolygon(currentPosition)
				currentPosition.move(Point2D(step_size*vector[0], step_size*vector[1]))
				path.append((currentPosition._x, currentPosition._y))
				print((currentPosition._x, currentPosition._y))
			# check step possibility
			L = PointsToLine2D(currentPosition, goalPoint)
			vector = (L.b, -L.a)
			currentPosition.move(Point2D(step_size*vector[0], step_size*vector[1]))
			path.append((currentPosition._x, currentPosition._y))
			print((currentPosition._x, currentPosition._y))
			if closestPolygon.computeDistancePointToPolygon(currentPosition)[0] < step_size:
				return "Failure: No possible path to reach goal point", path

		L = PointsToLine2D(currentPosition, goalPoint)
		vector = (L.b, -L.a)

		currentPosition.move(Point2D(step_size*vector[0], step_size*vector[1]))
		path.append((currentPosition._x, currentPosition._y))
		print((currentPosition._x, currentPosition._y))
	path.append(goal)
	return "Success", path

def main():
	start, goal, step_size, obstaclesList = parseInputs('input.txt')
	status, path = bug1Algorithm(start, goal, step_size, obstaclesList)
	print(status)
	import numpy as np
	from matplotlib import pyplot as plt
	plt.scatter(*np.array(path).T, s=4)
	plt.plot(start[0], start[1], 'ro', markersize=10)
	plt.plot(goal[0], goal[1],'go', markersize=10)
	plt.show()
	# print('\n'.join([str(x) for x in path]))

if __name__ == '__main__':
	main()