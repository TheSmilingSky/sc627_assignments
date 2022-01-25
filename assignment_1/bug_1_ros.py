#!/usr/bin/env python3

import rospy
import actionlib
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
from helper import Point2D, PointsToLine2D, Polygon2D
from bug_base import parseInputs
import math

class Bug1AlgoClass():

	def __init__(self, start, goal, step_size, obstaclesList, client, verbose = False):

		self.client = client

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
		self.currentPosition = self.LocalPosToPoint2D()

		self.closestPolygon = None
		self.closestDistance = float('inf')

		self.vector = (None, None)

		self.closestToGoalDist = float('inf')
		self.closestToGoalPoint = self.startPoint

		self.verbose = verbose

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

		self.path.append((self.currentPosition._x, self.currentPosition._y))

		
		print("localPos: ", self.localPos.pose_final.x, self.localPos.pose_final.y)

		# if self.verbose:
		# 	print(self.path[-1])


	def circumNavigation(self):

		print("begin circumNavigation")

		n = self.closestPolygon._n
		V = self.closestPolygon._V

		centroid = Point2D(sum([V[x][0] for x in range(n)])/n, sum([V[x][1] for x in range(n)])/n)

		initVector = (self.currentPosition-centroid)/self.currentPosition.euclideanDistance(centroid)

		circumnavProximityLimit = 50

		circumnavOrigin = Point2D(self.currentPosition._x, self.currentPosition._y)

		self.closestToGoalDist = self.goalPoint.euclideanDistance(self.currentPosition)
		self.closestToGoalPoint = self.currentPosition

		self.vector = self.closestPolygon.computeTangentVectorToPolygon(self.currentPosition)

		self.moveAndUpdatePath()

		print("First step in circumnav successful")

		circumNavLocalCnt = 1

		# while circumNavLocalCnt < circumnavProximityLimit or circumnavOrigin.euclideanDistance(self.currentPosition)>= self.step_size:
		while circumNavLocalCnt < circumnavProximityLimit or (initVector*newVector).coordinateSum()<0.7:

			print("In circumnav loop")

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

		print("localPos: ", self.localPos.pose_final.x, self.localPos.pose_final.y)

		while self.goalPoint.euclideanDistance(self.currentPosition) > self.step_size:

			self.findClosestPolygon()

			if self.closestDistance < 1.5*self.step_size:

				self.circumNavigation()
				
				# move to point closest to goal
				goalLine = PointsToLine2D(self.closestToGoalPoint, self.goalPoint)
				while goalLine.computeDistancePointToSegment(self.currentPosition) > self.step_size:
					
					print("Moving towards closest point")

					self.vector = self.closestPolygon.computeTangentVectorToPolygon(self.currentPosition)
					
					self.moveAndUpdatePath()

				# check step possibility
				L = PointsToLine2D(self.currentPosition, self.goalPoint)

				self.vector = (L.b, -L.a)

				self.moveAndUpdatePath()

				if self.closestPolygon.computeDistancePointToPolygon(self.currentPosition)[0] < self.step_size:
					return "Failure: No possible path to reach goal point", self.path

			print("Free Roam")

			L = PointsToLine2D(self.currentPosition, self.goalPoint)
			
			self.vector = (L.b, -L.a)

			self.moveAndUpdatePath()

		self.path.append((self.goalPoint._x, self.goalPoint._y))

		return "Success", self.path

if __name__ == '__main__':

	# ROS Initialization

	rospy.init_node('bug1', anonymous = True)
	client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
	client.wait_for_server()

	# Read Input File

	start, goal, step_size, obstaclesList = parseInputs('input.txt')

	algo = Bug1AlgoClass(start, goal, step_size, obstaclesList, client, 'False')
	status, path = algo.bug1Algorithm()
	print(status)
	import numpy as np
	from matplotlib import pyplot as plt
	plt.scatter(*np.array(path).T, s=4)
	plt.plot(start[0], start[1], 'ro', markersize=10)
	plt.plot(goal[0], goal[1],'go', markersize=10)
	for i in range(len(obstaclesList)):
		t1 = plt.Polygon(obstaclesList[i], color = 'red', fill = False)
		plt.gca().add_patch(t1)
	plt.show()



