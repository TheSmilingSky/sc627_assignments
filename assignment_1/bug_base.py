#!/usr/bin/env python3

import rospy
import actionlib
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
from helper import Point2D, PointsToLine2D, Polygon2D
from dataHandler import parseInputs, saveData
import math
import time

class BugBaseAlgoClass():

	def __init__(self, start, goal, step_size, obstaclesList, client):

		self.client = client
		self.localAlgoStartTime = time.time()

		self.startPoint = Point2D(start[0], start[1])
		self.goalPoint = Point2D(goal[0], goal[1])

		self.localPos = MoveXYResult()
		self.localPos.pose_final.x = start[0]
		self.localPos.pose_final.y = start[1]
		self.localPos.pose_final.theta = 0

		self.localPosOld = self.localPos

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


	def LocalPosToPoint2D(self):

		x = self.localPos.pose_final.x
		y = self.localPos.pose_final.y

		return Point2D(x, y)


	def findClosestObstacle(self):

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

		self.localPosOld = self.localPos

		self.client.send_goal(self.localGoal)
		self.client.wait_for_result()

		self.localPos = self.client.get_result()

		self.currentPosition = self.LocalPosToPoint2D()

		self.storeData()
		
		print("localResult: ", self.localPos.pose_final.x, self.localPos.pose_final.y, self.localPos.pose_final.theta - 2*math.pi*(self.localPos.pose_final.theta > math.pi),'\n')


	def moveTowardsGoal(self):

		print("Moving towards goal")

		L = PointsToLine2D(self.currentPosition, self.goalPoint)
		
		self.vector = (L.b, -L.a)

		self.moveAndUpdatePath()


	def isNearObstacle(self):

		return self.closestDistance < 2*self.step_size


	def storeData(self):

		self.goalDistancesFromBugLog.append((time.time() - self.localAlgoStartTime, self.goalPoint.euclideanDistance(self.currentPosition)))

		self.path.append((self.currentPosition._x, self.currentPosition._y))

		self.pathLength += self.currentPosition.euclideanDistance(Point2D(self.localPosOld.pose_final.x, self.localPosOld.pose_final.y))

	
	def goalCondition(self):

		return self.goalPoint.euclideanDistance(self.currentPosition) < self.step_size

	
	def getAlgoOutcome(self, fail = False):

		if fail:

			return "Failure: No possible path to reach goal point", self.path, self.pathLength, self.goalDistancesFromBugLog
		
		return "Success", self.path, self.pathLength, self.goalDistancesFromBugLog


	def bugBaseAlgorithm(self):

		while not self.goalCondition():

			self.findClosestObstacle()

			if self.isNearObstacle():

				return self.getAlgoOutcome(fail = True)

			self.moveTowardsGoal()

		self.storeData()

		return self.getAlgoOutcome(fail = False)

def ROSInit():

	# ROS Initialization
	rospy.init_node('bug', anonymous = True)
	client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
	client.wait_for_server()

	return client

def main():

	# start timer
	startTime = time.time()

	start, goal, step_size, obstaclesList = parseInputs('input.txt')
	
	client = ROSInit()

	algo = BugBaseAlgoClass(start, goal, step_size, obstaclesList, client)
	
	status, path, pathLength, goalDistanceLog = algo.bugBaseAlgorithm()
	
	endTime = time.time()

	print(f"Status: {status}")

	print('\n#############################\nResults saved to data folder\n#############################\n')

	saveData(path, start, goal, obstaclesList, goalDistanceLog, startTime, endTime, pathLength, status, algo = 'BugBase')


if __name__ == '__main__':

	main()