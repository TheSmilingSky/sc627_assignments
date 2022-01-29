import numpy as np
import matplotlib.pyplot as plt

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

def saveData(path, start, goal, obstaclesList, goalDistanceLog, startTime, endTime, pathLength, status, algo):

	np.savetxt(f'data/{algo}/output_{algo}.txt', path)
	np.savetxt(f'data/{algo}/goalDistFromBug.txt', goalDistanceLog)

	plt.figure()
	plt.grid()
	plt.xlabel('Time (s)')
	plt.ylabel('Distance from Goal (m)')
	plt.plot(*np.array(goalDistanceLog).T)
	plt.savefig(f'data/{algo}/goalDistFromBug.png')

	plt.figure()
	plt.scatter(*np.array(path).T, s=4)
	plt.plot(start[0], start[1], 'ro', markersize=10)
	plt.plot(goal[0], goal[1],'go', markersize=10)

	for i in range(len(obstaclesList)):
		t1 = plt.Polygon(obstaclesList[i], color = 'red', fill = False)
		plt.gca().add_patch(t1)
	plt.savefig(f'data/{algo}/path.png')

	with open(f'data/{algo}/info.txt', 'w+') as f:
		f.write(f'Result of the motion planning algorithm: {status}\n')
		f.write(f'The total length of path taken: {pathLength}\n')
		f.write(f"Time taken by the algorithm is: {endTime - startTime}")