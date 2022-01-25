#!/usr/bin/env python3

import rospy
import actionlib
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
from helper import Point2D, PointsToLine2D, Polygon2D
from bug_base import parseInputs
import math
import numpy as np
from matplotlib import pyplot as plt
import time

class Bug1AlgoClass():

	def __init__(self, start, goal, step_size, obstaclesList, client):

		self.client = client
		self.localAlgoStartTime = time.time()

		self.startPoint = Point2D(start[0], start[1])
		self.goalPoint = Point2D(goal[0], goal[1])

		self.localPos = MoveXYResult()
		self.localPos.pose_final.x = start[0]
		self.localPos.pose_final.y = start[1]
		self.localPos.pose_final.theta = 0

		self.localGoal = MoveXYGoal()

		self.step_size = step_size
		self.obstaclesList = obstaclesList

		self.path = []
		self.pathLength = 0
		self.currentPosition = self.LocalPosToPoint2D()
		self.goalDistancesFromBugLog = [(time.time() - self.localAlgoStartTime, self.goalPoint.euclideanDistance(self.currentPosition))]

		self.closestPolygon = None
		self.closestDistance = float('inf')

		self.vector = (None, None)

		self.closestToGoalDist = float('inf')
		self.closestToGoalPoint = self.startPoint	


	def LocalPosToPoint2D(self):

		x = self.localPos.pose_final.x
		y = self.localPos.pose_final.y

		return Point2D(x, y)


	def findClosestPolygon(self):

		self.closestDistance = float('inf')

		for obstacle in self.obstaclesList:

			polygon = Polygon2D(len(obstacle), obstacle)
			distance, _, _ = polygon.computeDistancePointToPolygon(self.currentPosition)

			if distance < self.closestDistance:
				self.closestDistance = distance
				self.closestPolygon = polygon


	def moveAndUpdatePath(self):

		self.localGoal.pose_dest.x = self.currentPosition._x + self.step_size*self.vector[0]
		self.localGoal.pose_dest.y = self.currentPosition._y + self.step_size*self.vector[1]
		self.localGoal.pose_dest.theta = math.atan2(self.localGoal.pose_dest.y - self.localPos.pose_final.y, self.localGoal.pose_dest.x - self.localPos.pose_final.x)

		print("localGoal: ", self.localGoal.pose_dest.x, self.localGoal.pose_dest.y, self.localGoal.pose_dest.theta)

		self.client.send_goal(self.localGoal)
		self.client.wait_for_result()

		self.localPos = self.client.get_result()

		self.currentPosition = self.LocalPosToPoint2D()

		self.goalDistancesFromBugLog.append((time.time() - self.localAlgoStartTime, self.goalPoint.euclideanDistance(self.currentPosition)))

		self.path.append((self.currentPosition._x, self.currentPosition._y))

		self.pathLength += self.currentPosition.euclideanDistance(Point2D(self.localGoal.pose_dest.x, self.localGoal.pose_dest.y))
		
		print("localResult: ", self.localPos.pose_final.x, self.localPos.pose_final.y, self.localPos.pose_final.theta,'\n')


	def getCentroid(self):

		n = self.closestPolygon._n
		V = self.closestPolygon._V

		centroid = Point2D(sum([V[x][0] for x in range(n)])/n, sum([V[x][1] for x in range(n)])/n)

		return centroid


	def circumNavigation(self):

		print("\n###########################\nBegin circumNavigation\n###########################\n")

		centroid = self.getCentroid()

		initVector = (self.currentPosition-centroid)/self.currentPosition.euclideanDistance(centroid)

		circumnavProximityLimit = 50

		circumnavOrigin = Point2D(self.currentPosition._x, self.currentPosition._y)

		self.closestToGoalDist = self.goalPoint.euclideanDistance(self.currentPosition)
		self.closestToGoalPoint = self.currentPosition

		self.vector = self.closestPolygon.computeTangentVectorToPolygon(self.currentPosition)

		self.moveAndUpdatePath()

		circumNavLocalCnt = 1

		# while circumNavLocalCnt < circumnavProximityLimit or circumnavOrigin.euclideanDistance(self.currentPosition)>= self.step_size:
		while circumNavLocalCnt < circumnavProximityLimit or (initVector*newVector).coordinateSum()<0.9:

			print("In circumnav loop")

			localGoalDist = self.goalPoint.euclideanDistance(self.currentPosition)

			if localGoalDist < self.closestToGoalDist:

				self.closestToGoalDist = localGoalDist
				self.closestToGoalPoint = Point2D(self.currentPosition._x, self.currentPosition._y)

			self.vector = self.closestPolygon.computeTangentVectorToPolygon(self.currentPosition)

			self.moveAndUpdatePath()

			newVector = (self.currentPosition-centroid)/self.currentPosition.euclideanDistance(centroid)

			circumNavLocalCnt += 1


	def bug1Algorithm(self):

		print("localPos: ", self.localPos.pose_final.x, self.localPos.pose_final.y, self.localPos.pose_final.theta)

		while self.goalPoint.euclideanDistance(self.currentPosition) > self.step_size:

			self.findClosestPolygon()

			if self.closestDistance < 2*self.step_size:

				self.circumNavigation()

				print("\n###########################\nBegin moving towards closest point to goal on obstacle\n###########################\n")
				
				# move to point closest to goal
				goalLine = PointsToLine2D(self.closestToGoalPoint, self.goalPoint)
				while goalLine.computeDistancePointToSegment(self.currentPosition) > 2*self.step_size:
					
					print("Moving towards closest point")

					self.vector = self.closestPolygon.computeTangentVectorToPolygon(self.currentPosition)
					
					self.moveAndUpdatePath()

				# check goal feasibility
				centroid = self.getCentroid()

				initVector = (centroid - self.currentPosition)/self.currentPosition.euclideanDistance(centroid)
				goalVector = (self.goalPoint - self.currentPosition)/self.currentPosition.euclideanDistance(self.goalPoint)

				if (initVector*goalVector).coordinateSum()>0:
					return "Failure: No possible path to reach goal point", self.path, self.pathLength, self.goalDistancesFromBugLog

				# move a few steps away
				for _ in range(3):

					L = PointsToLine2D(self.currentPosition, self.goalPoint)

					self.vector = (L.b, -L.a)

					self.moveAndUpdatePath()

			print("Moving towards goal")

			L = PointsToLine2D(self.currentPosition, self.goalPoint)
			
			self.vector = (L.b, -L.a)

			self.moveAndUpdatePath()

		self.goalDistancesFromBugLog.append((time.time() - self.localAlgoStartTime, self.goalPoint.euclideanDistance(self.currentPosition)))

		self.path.append((self.goalPoint._x, self.goalPoint._y))

		return "Success", self.path, self.pathLength, self.goalDistancesFromBugLog


def saveData(path, start, goal, goalDistanceLog):

	np.savetxt('data/output_bug1.txt', path)
	np.savetxt('data/goalDistFromBug.txt', path)

	plt.figure()
	plt.grid()
	plt.xlabel('Time (s)')
	plt.ylabel('Distance from Goal (m)')
	plt.plot(*np.array(goalDistanceLog).T)
	plt.savefig('data/goalDistFromBug.png')

	plt.figure()
	plt.scatter(*np.array(path).T, s=4)
	plt.plot(start[0], start[1], 'ro', markersize=10)
	plt.plot(goal[0], goal[1],'go', markersize=10)

	for i in range(len(obstaclesList)):
		t1 = plt.Polygon(obstaclesList[i], color = 'red', fill = False)
		plt.gca().add_patch(t1)
	plt.savefig('data/path.png')


if __name__ == '__main__':

	# start timer
	startTime = time.time()

	# ROS Initialization
	rospy.init_node('bug1', anonymous = True)
	client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
	client.wait_for_server()

	# Read Input File
	start, goal, step_size, obstaclesList = parseInputs('input.txt')

	# Run Bug1 Algorithm
	algo = Bug1AlgoClass(start, goal, step_size, obstaclesList, client)
	status, path, pathLength, goalDistanceLog = algo.bug1Algorithm()

	print('\n#############################\nResults\n#############################\n')

	# Print Status
	print(f'Result of the motion planning algorithm: {status}')

	# Save Generated Data
	print('Saving generated data')
	saveData(path, start, goal, goalDistanceLog)

	# Print Path Length
	print(f'The total length of path taken: {pathLength}')

	# stopTimer
	endTime = time.time()

	# Print time taken by algorithm
	print(f"Time taken by the algorithm is: {endTime - startTime}")