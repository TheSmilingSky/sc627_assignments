from helper import Point2D, PointsToLine2D, Polygon2D

def parseInputs(filename):
	with open(filename, 'r') as f:
		data = f.read().split('\n\n')

		missionData = data[0].split('\n')
		start_x, start_y = [float(x) for x in missionData[0].split(', ')]
		goal_x, goal_y = [float(x) for x in missionData[1].split(', ')]
		step_size = float(missionData[2])

		start = (start_x, start_y)
		goal = (goal_x, goal_y)

		obstaclesData = data[1:]
		obstaclesList = []

		for i, obsData in enumerate(obstaclesData):

			obstaclesList.append([])
			obsData = obsData.split('\n')

			for od in obsData:

				xx, yy = [float(x) for x in od.split(', ')]
				obstaclesList[i].append([xx,yy])

		return start, goal, step_size, obstaclesList

# print(parseInputs('input.txt'))

def bugBaseAlgorithm(start, goal, step_size, obstaclesList):

	startPoint = Point2D(start[0], start[1])
	goalPoint = Point2D(goal[0], goal[1])

	currentPosition = startPoint

	path = [start]

	while goalPoint.euclideanDistance(currentPosition) > step_size:

		# find polygon closest to currentPosition
		closestPolygon = None
		closestDistance = float('inf')

		for obstacle in obstaclesList:
			polygon = Polygon2D(len(obstacle), obstacle)
			distance, _, _ = polygon.computeDistancePointToPolygon(currentPosition)
			if distance < closestDistance:
				closestDistance = distance
				closestPolygon = polygon

		if closestDistance < step_size:
			return "Failure: There is an obstacle lying between start and goal", path

		L = PointsToLine2D(currentPosition, goalPoint)
		vector = (L.b, -L.a)

		currentPosition.move(Point2D(step_size*vector[0], step_size*vector[1]))
		path.append((currentPosition._x, currentPosition._y))
	path.append(goal)
	return "Success", path

def main():

	start, goal, step_size, obstaclesList = parseInputs('input.txt')
	status, path = bugBaseAlgorithm(start, goal, step_size, obstaclesList)
	print(status)
	print('\n'.join([str(x) for x in path]))

if __name__ == '__main__':
	main()


