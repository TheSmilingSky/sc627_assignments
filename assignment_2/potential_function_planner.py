#!/usr/bin/env python3
import sys
sys.path.append("..")
from dataHandler import parseInputs, saveData
from assignment_1.helper import Point2D, PointsToLine2D, Polygon2D
import time
import rospy
import actionlib
from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  

class PotentialBasedPlanner():
	def __init__(self, start, goal, step_size, obstaclesList, client = None):

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
		self.plan = []
		self.pathLength = 0
		self.currentPosition = self.LocalPosToPoint2D()
		self.goalDistancesFromBugLog = [(time.time() - self.localAlgoStartTime, self.goalPoint.euclideanDistance(self.currentPosition))]

		self.closestPolygon = None
		self.closestDistance = float('inf')

		self.gradient = Point2D(1,1)
		self.attractiveGradient = Point2D(0,0)
		self.repulsiveGradient = Point2D(0,0)

		self.dstar = 1.5
		self.chi = 0.8

		self.Qstar = 2
		self.eta = 0.8


	def LocalPosToPoint2D(self):

		x = self.localPos.pose_final.x
		y = self.localPos.pose_final.y

		return Point2D(x, y)


	def goalCondition(self):
		
		# return abs(self.gradient.euclideanDistance(None)) < 0.1
		return self.currentPosition.euclideanDistance(self.goalPoint) < 0.1
		

	def computeGradient(self):

		if self.goalPoint.euclideanDistance(self.currentPosition) <= self.dstar:
			self.attractiveGradient = (self.goalPoint - self.currentPosition)*self.chi
		else:
			self.attractiveGradient = (self.goalPoint - self.currentPosition)*self.dstar*self.chi / self.goalPoint.euclideanDistance(self.currentPosition)

		self.repulsiveGradient = Point2D(0,0)

		for obstacle in self.obstaclesList:

			polygon = Polygon2D(len(obstacle), obstacle)
			distance, _, _ = polygon.computeDistancePointToPolygon(self.currentPosition)
			
			if distance < self.Qstar:
				
				normal = polygon.computeNormalVectorToPolygon(self.currentPosition)
				
				self.repulsiveGradient =  self.repulsiveGradient + (Point2D(normal[0], normal[1])) * self.eta * (-1/self.Qstar + 1/distance) * (1/distance**2)

		self.gradient = self.attractiveGradient + self.repulsiveGradient

		self.gradient /= self.gradient.euclideanDistance(None)


	def storeData(self):

		self.goalDistancesFromBugLog.append((time.time() - self.localAlgoStartTime, self.goalPoint.euclideanDistance(self.currentPosition)))

		self.path.append((self.currentPosition._x, self.currentPosition._y))

		self.pathLength += self.currentPosition.euclideanDistance(Point2D(self.localPosOld.pose_final.x, self.localPosOld.pose_final.y))


	def moveAndUpdatePath(self, shoot = False):

		if not shoot:
			self.computeGradient()

		self.localGoal.pose_dest.x = self.currentPosition._x + self.step_size*self.gradient._x
		self.localGoal.pose_dest.y = self.currentPosition._y + self.step_size*self.gradient._y
		self.localGoal.pose_dest.theta = math.atan2(self.localGoal.pose_dest.y - self.localPos.pose_final.y, self.localGoal.pose_dest.x - self.localPos.pose_final.x)

		print("localGoal: ", self.localGoal.pose_dest.x, self.localGoal.pose_dest.y, self.localGoal.pose_dest.theta)

		self.localPosOld = self.localPos

		self.plan.append((self.localGoal.pose_dest.x, self.localGoal.pose_dest.y))

		self.client.send_goal(self.localGoal)
		self.client.wait_for_result()

		self.localPos = self.client.get_result()

		self.currentPosition = self.LocalPosToPoint2D()

		self.storeData()
		
		print("localResult: ", self.localPos.pose_final.x, self.localPos.pose_final.y, self.localPos.pose_final.theta - 2*math.pi*(self.localPos.pose_final.theta > math.pi),'\n')

	
	def moveAndUpdatePathPython(self, shoot = False):

		if not shoot:
			self.computeGradient()

		self.plan.append((self.currentPosition._x, self.currentPosition._y))

		self.currentPosition.move(Point2D(self.step_size*self.gradient._x, self.step_size*self.gradient._y))

		self.path.append((self.currentPosition._x, self.currentPosition._y))

	def shootTowardsGoal(self):
		
		for i in range(3):
			self.gradient = self.goalPoint - self.currentPosition
			self.gradient /= self.gradient.euclideanDistance()
			print('shoot')

			print(self.currentPosition._x, self.currentPosition._y)
			print(self.gradient._x, self.gradient._y)
		
			self.moveAndUpdatePath(shoot = True)

		print()


	def planner(self):

		while not self.goalCondition():
			
			self.moveAndUpdatePath()
			# self.moveAndUpdatePathPython()

			if len(self.path)>3:
				for i in range(3):
					if self.currentPosition.euclideanDistance(Point2D(self.path[-2-i][0], self.path[-2-i][1])) < 0.04:
						self.shootTowardsGoal()

		return "Success", self.path, self.plan

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
	# client = None

	algo = PotentialBasedPlanner(start, goal, step_size, obstaclesList, client)
	
	status, path, plan = algo.planner()
	
	endTime = time.time()

	print(f"Status: {status}")

	print('\n#############################\nResults saved to data folder\n#############################\n')

	saveData(path, plan, start, goal, obstaclesList, startTime, endTime, status)


if __name__ == '__main__':

	main()