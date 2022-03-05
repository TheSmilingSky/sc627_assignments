import math
import numpy as np

class Point2D():

	def __init__(self, x, y):
		
		self._x = x
		self._y = y


	def __add__(self, other):
		return Point2D(self._x + other._x, self._y + other._y)


	def __sub__(self, other):
		return Point2D(self._x - other._x, self._y - other._y)


	def __mul__(self, other):

		if isinstance(other, (int, float)):
			return Point2D(self._x * other, self._y * other)

		return Point2D(self._x * other._x, self._y * other._y)


	def __truediv__(self, num):
		return Point2D(self._x/num, self._y/num)


	def move(self, other):

		self._x += other._x
		self._y += other._y


	def reverse(self):
		return Point2D(-self._x, -self._y)


	def coordinateSum(self, other = None):

		if not other:
			return self._x + self._y

		return self._x - other._x + self._y - other._y


	def euclideanDistance(self, other = None):
		
		if not other:
			return np.sqrt(self._x**2 + self._y**2)

		return np.sqrt((self._x - other._x)**2 + (self._y - other._y)**2)


class PointsToLine2D():

	def __init__(self, p1, p2):
		
		self._p1 = p1
		self._p2 = p2
		self.norm = p1.euclideanDistance(p2)
		self.a, self.b, self.c = self.computeLineThroughTwoPoints()
		self._params = [self.a, self.b, self.c]


	def computeLineThroughTwoPoints(self):
		
		if self.norm < 10**(-8):
			return [0, 0, 0]

		self.a = (self._p1._y - self._p2._y)/self.norm
		self.b = (self._p2._x - self._p1._x)/self.norm
		self.c = (self._p1._x*self._p2._y - self._p2._x*self._p1._y)/self.norm

		return [self.a, self.b, self.c]


	def computeDistancePointToLine(self, p):
		
		return abs(self.a*p._x + self.b*p._y + self.c)


	def computeDistancePointToSegment(self, p):

		factor = (p._x - self._p1._x)*self.b - (p._y - self._p1._y)*self.a
		factor /= self.norm

		if factor<0:
			return p.euclideanDistance(self._p1)
		elif factor>1:
			return p.euclideanDistance(self._p2)
		else:
			return self.computeDistancePointToLine(p)


class Polygon2D():
	
	def __init__(self, n, V):

		self._n = n
		self._V = V


	def isCounterClockwise(self):

		signedArea = 0

		for i in range(self._n):
			signedArea += (self._V[i%self._n][0] - self._V[(i+1)%self._n][0])*(self._V[i%self._n][1] + self._V[(i+1)%self._n][1])
		
		return signedArea > 0


	def computeDistancePointToPolygon(self, p):
		
		minDist = p.euclideanDistance(Point2D(self._V[0][0], self._V[0][1]))
		dists = []
		minDistIdx = 0

		for i in range(self._n):

			p1 = Point2D(self._V[i%self._n][0], self._V[i%self._n][1])
			p2 = Point2D(self._V[(i+1)%self._n][0], self._V[(i+1)%self._n][1])
			L = PointsToLine2D(p1, p2)

			dist = L.computeDistancePointToSegment(p)
			dists.append(dist)

			if dist<minDist:
				minDist = dist
				minDistIdx = i

		if dists[(minDistIdx+1)%self._n] == minDist:
			return minDist, 'V', self._V[(minDistIdx+1)%self._n]
		elif dists[(minDistIdx-1)%self._n] == minDist:
			return minDist, 'V', self._V[0]

		return minDist, 'E', (self._V[minDistIdx%self._n], self._V[(minDistIdx+1)%self._n])


	def computeTangentVectorToPolygon(self, p):

		minDist, minType, minData = self.computeDistancePointToPolygon(p)

		if minType == 'V':

			p1 = Point2D(minData[0], minData[1])
			xx = -(p._y - p1._y) + 0.3*(p1._x - p._x)
			yy = (p._x - p1._x) + 0.3*(p1._y - p._y)

		elif minType == 'E':

			order = 0.1*(2*self.isCounterClockwise() - 1)
			p1 = Point2D(minData[0][0], minData[0][1])
			p2 = Point2D(minData[1][0], minData[1][1])

			xx = order*(p2._x - p1._x)
			yy = order*(p2._y - p1._y)

		norm = np.sqrt(xx**2 + yy**2)

		return (xx/norm, yy/norm)

	def computeNormalVectorToPolygon(self, p):

		minDist, minType, minData = self.computeDistancePointToPolygon(p)

		if minType == 'V':

			p1 = Point2D(minData[0], minData[1])
			xx = (p._x - p1._x) + 0.3*(p1._y - p._y)
			yy = (p._y - p1._y) - 0.3*(p1._x - p._x)

		elif minType == 'E':

			order = (2*self.isCounterClockwise() - 1)
			p1 = Point2D(minData[0][0], minData[0][1])
			p2 = Point2D(minData[1][0], minData[1][1])

			xx = (p2._y - p1._y)
			yy = -(p2._x - p1._x)

		norm = np.sqrt(xx**2 + yy**2)

		return (xx/norm, yy/norm)


def main():

	p1 = Point2D(0,1)
	p2 = Point2D(1,0)
	print(p1.euclideanDistance(p2))
	print("@@@@@@@@@@@@@@@@@@@@@")

	L = PointsToLine2D(p1,p2)
	print(L._params)
	print(L.computeDistancePointToLine(Point2D(0,0)))
	print(L.computeDistancePointToLine(Point2D(1,-1)))
	print(L.computeDistancePointToLine(Point2D(1,0)))
	print(L.computeDistancePointToLine(Point2D(2,-1)))
	print("@@@@@@@@@@@@@@@@@@@@@")

	print(L.computeDistancePointToSegment(Point2D(0,0)))
	print(L.computeDistancePointToSegment(Point2D(1,-1)))
	print(L.computeDistancePointToSegment(Point2D(1,0)))
	print(L.computeDistancePointToSegment(Point2D(2,-1)))
	print("@@@@@@@@@@@@@@@@@@@@@")

	sq = [[0,0],[0,1],[1,1],[1,0]]
	P = Polygon2D(4,sq)
	print(P.isCounterClockwise())
	print(P.computeDistancePointToPolygon(Point2D(2,2)))
	print(P.computeTangentVectorToPolygon(Point2D(2,2)))
	print(P.computeDistancePointToPolygon(Point2D(2,1)))
	print(P.computeTangentVectorToPolygon(Point2D(2,1)))
	print(P.computeDistancePointToPolygon(Point2D(0.5,1.5)))
	print(P.computeTangentVectorToPolygon(Point2D(0.5,1.5)))


if __name__=='__main__':
	main()