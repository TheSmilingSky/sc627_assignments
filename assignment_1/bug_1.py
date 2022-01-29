#!/usr/bin/env python3

import math
import time
import rospy
import actionlib
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
from helper import Point2D, PointsToLine2D, Polygon2D
from dataHandler import parseInputs, saveData
from bug_base import BugBaseAlgoClass, ROSInit

class Bug1AlgoClass(BugBaseAlgoClass):

	def __init__(self, start, goal, step_size, obstaclesList, client):

		super().__init__(start, goal, step_size, obstaclesList, client)

		self.closestToGoalDist = float('inf')
		self.closestToGoalPoint = self.startPoint	


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


	def moveToObstaclePointClosestToGoal(self):

		print("\n###########################\nBegin moving towards closest point to goal on obstacle\n###########################\n")
				
		goalLine = PointsToLine2D(self.closestToGoalPoint, self.goalPoint)

		while goalLine.computeDistancePointToSegment(self.currentPosition) > 2*self.step_size:
			
			print("Moving towards closest point")

			self.vector = self.closestPolygon.computeTangentVectorToPolygon(self.currentPosition)
			
			self.moveAndUpdatePath()


	def isGoalFeasible(self):

		centroid = self.getCentroid()

		initVector = (centroid - self.currentPosition)/self.currentPosition.euclideanDistance(centroid)
		goalVector = (self.goalPoint - self.currentPosition)/self.currentPosition.euclideanDistance(self.goalPoint)

		return (initVector*goalVector).coordinateSum()<0


	def moveLittleAwayFromObstacle(self):

		for _ in range(3):

			print("Moving a little away from obstacle")

			L = PointsToLine2D(self.currentPosition, self.goalPoint)

			self.vector = (L.b, -L.a)

			self.moveAndUpdatePath()


	def bug1Algorithm(self):

		while not self.goalCondition():

			self.findClosestObstacle()

			if self.isNearObstacle():

				self.circumNavigation()

				self.moveToObstaclePointClosestToGoal()

				if not self.isGoalFeasible():

					return self.getAlgoOutcome(fail = True)

				self.moveLittleAwayFromObstacle()

			self.moveTowardsGoal()

		self.storeData()

		return self.getAlgoOutcome(fail = False)


def main():

	# start timer
	startTime = time.time()

	start, goal, step_size, obstaclesList = parseInputs('input.txt')
	
	client = ROSInit()

	algo = Bug1AlgoClass(start, goal, step_size, obstaclesList, client)
	
	status, path, pathLength, goalDistanceLog = algo.bug1Algorithm()
	
	endTime = time.time()

	print(f"Status: {status}")

	print('\n#############################\nResults saved to data folder\n#############################\n')

	saveData(path, start, goal, obstaclesList, goalDistanceLog, startTime, endTime, pathLength, status, algo = 'Bug1')


if __name__=='__main__':

	main()