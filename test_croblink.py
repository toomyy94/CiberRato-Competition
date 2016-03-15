import math

from croblink import *

class MyRob(CRobLink):

	def run(self):
		if rob.status != 0:
			print "Connection refused or error"
			quit()

		self.counter = 0	#para a funcao determineAction
		self.avoid = False
		self.start_saved = False
		self.path = []
		state = 'stop'
		stoppedState = 'run'

		while(True):
			self.readSensors()

			if state == 'return' and (distance(float(self.measures.x - self.startPos[0]), float(self.measures.y - self.startPos[1])) < 1):
				print self.robName + " exiting"
				self.finish()
				quit()

			if (state == 'stop' and self.measures.start):
				state = stoppedState

			if (state != 'stop' and self.measures.stop):
				stoppedState = state
				state = 'stop'

			if (state == 'run'):
				if not self.start_saved:
					if self.measures.gpsReady:
						self.startPos = (self.measures.x, self.measures.y)
						self.path = [(self.measures.x, self.measures.y)]
						self.start_saved = True

				if self.measures.visitingLed:
					state = 'return'

				if self.measures.groundReady and self.measures.ground:
					self.setVisitingLed(True)
					print self.robName + " visited target area"

				else:
					(lPow, rPow) = self.determineAction("run")
					self.driveMotors(lPow, rPow)

			if (state == 'return'):
				(lPow, rPow) = self.determineAction("return")
				self.driveMotors(lPow, rPow)


	def determineAction(self, state):

		center_id = 0
		left_id = 1
		right_id = 2
		back_id = 3
		center = left = right = 0

		if self.measures.irSensorReady[left_id]:
			left = self.measures.irSensor[left_id]

		if self.measures.irSensorReady[right_id]:
			right = self.measures.irSensor[right_id]

		if self.measures.irSensorReady[center_id]:
			center = self.measures.irSensor[center_id]

		beaconReady = self.measures.beaconReady
		if (beaconReady):
			(beaconVisible, beaconDir) = self.measures.beacon

		if (self.measures.groundReady):
			ground = self.measures.ground

		if (self.measures.collisionReady):
			collision = self.measures.collision

		if (center > 4.5 or right > 4.5 or left > 4.5 or collision):
			#maior numero do sensor + proximo do obstaculo onde se encontra

			if (self.counter % 400 < 200):
				lPow = 0.06
				rPow = -0.06

			else:
				lPow = -0.06
				rPow = 0.06
				
			self.avoid = True
			
		elif right > 1.5:
			lPow = 0.0
			rPow = 0.05
			self.avoid = True
		elif left > 1.5:
			lPow = 0.05
			rPow = 0.0
			self.avoid = True

		else:
			#Quando ta a ir para o farol
			if (state == 'run'):			
				if (beaconReady and beaconVisible and beaconDir > 20.0):
					lPow = 0.0
					rPow = 0.1

				elif (beaconReady and beaconVisible and beaconDir < -20.0):
					lPow = 0.1
					rPow = 0.0

				else:
					if(self.avoid):
						if self.measures.gpsReady:
							self.path[0:0] = [(self.measures.x, self.measures.y)]
					
					(listaF, boolean) = darMeiaVolta(self.measures.x, self.measures.y, self.path[1:])
					print(listaF)
					if(boolean):
						self.path = listaF
						lPow = -0.5
						rPow = 0.5
					elif(center > 0 and right > 0 and left == 0):
						lPow = 0.0 
						rPow = 0.1
					elif(center > 0 and left > 0 and right == 0):
						lPow = 0.1
						rPow = 0.0
					else:
						lPow = 0.1
						rPow = 0.1
					self.avoid = False

			#Para voltar para tras
			elif (state == 'return'):		
				if self.measures.gpsReady and self.measures.compassReady:
					if(distance((self.measures.x - self.path[0][0]), (self.measures.y - self.path[0][1])) < 1):
						self.path = self.path[1:]				
					compass = self.measures.compass				
					(lPow, rPow) = decision(compass, self.measures.x, self.path[0][0], self.measures.y, self.path[0][1])
					
		self.counter += 1
		return (lPow, rPow)

def calculaAngulo(x,y):
	if y ==0:
		y = 0.01
	return math.degrees(math.atan(float(x)/y))
	
def distance(x, y):
	return float((x**2 + y**2)**(float(1)/2))

def darMeiaVolta(x, y, lista):
	if lista == []:
		return ([], False)
		
	if distance(x-lista[0][0], y - lista[0][1]) < 1:
		return (lista, True)
	
	return darMeiaVolta(x,y,lista[1:])	
	
def decision(compass, x1, x2, y1, y2):	
	if(x1 >= x2 and y1 >= y2):
		alfa = (-1 * calculaAngulo((x1 - x2),(y1-y2))) - 90
		if(compass > 90 and compass < 180):
			lPow = 0.0
			rPow = 0.1
		elif(compass < 0 and compass > -90):
			lPow = 0.1
			rPow = 0.0
		elif(compass > 0 and compass < 90):
			lPow = 0.1
			rPow = -0.1
		else:		
			if (compass - alfa < -20.0):
				lPow = 0.0
				rPow = 0.1
			elif(compass - alfa > 20.0):
				lPow = 0.1
				rPow = 0.0
			else:
				lPow = 0.1
				rPow = 0.1
	elif(x1 >= x2 and y1 <= y2):
		alfa = calculaAngulo((x1 - x2),(y2-y1)) + 90
		if(compass < -90 and compass > -180):
			lPow = 0.1
			rPow = 0.0
		elif(compass < 0 and compass > -90):
			lPow = -0.1
			rPow = 0.1
		elif(compass > 0 and compass < 90):
			lPow = 0.0
			rPow = 0.1
		else:		
			if (compass - alfa < -20.0):
				lPow = 0.0
				rPow = 0.1
			elif(compass - alfa > 20.0):
				lPow = 0.1
				rPow = 0.0
			else:
				lPow = 0.1
				rPow = 0.1
	elif(x1 <= x2 and y1 >= y2):
		alfa = -1 * (90 - calculaAngulo((x1 - x2),(y2-y1)))
		if(compass > 90 and compass < 180):
			lPow = -0.1
			rPow = 0.1
		elif(compass < -90 and compass > -180):
			lPow = 0.0
			rPow = 0.1
		elif(compass > 0 and compass < 90):
			lPow = 0.1
			rPow = 0.0
		else:		
			if (compass - alfa < -20.0):
				lPow = 0.0
				rPow = 0.1
			elif(compass - alfa > 20.0):
				lPow = 0.1
				rPow = 0.0
			else:
				lPow = 0.1
				rPow = 0.1
	elif(x1 <= x2 and y1 <= y2):
		alfa = 90 - calculaAngulo((x2 - x1),(y1-y2))
		if(compass > 90 and compass < 180):
			lPow = 0.1
			rPow = 0.0
		elif(compass < 0 and compass > -90):
			lPow = 0.0
			rPow = 0.1
		elif(compass < -90 and compass > -180):
			lPow = 0.1
			rPow = -0.1
		else:		
			if (compass - alfa < -20.0):
				lPow = 0.0
				rPow = 0.1
			elif(compass - alfa > 20.0):
				lPow = 0.1
				rPow = 0.0
			else:
				lPow = 0.1
				rPow = 0.1
			
	return (lPow, rPow)
		
		
rob = MyRob("AA",3,"localhost")
rob.run()
