#Patricia Salome Rocha Gomes - 50141
#Luis Andre Fortuna Parra Afonso - 60412
#Mario Jorge Rodrigues Pina - 65292
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
		self.parede = 'Nenhuma'
		self.prev_ground = 99
		state = 'stop'
		stoppedState = 'run'
		self.inicio_parede = None

		while(True):
			self.readSensors()
			
			if (self.measures.groundReady):
				if self.measures.ground !=self.prev_ground:
					print state,"ground=",self.measures.ground
					self.prev_ground = self.measures.ground
 
			if self.measures.endLed:
				print self.robName + " exiting"
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
					self.setVisitingLed(False)
					self.setReturningLed(True)
				
				if self.measures.returningLed:
					state='return'

				if not self.measures.visitingLed and not self.measures.returningLed and self.measures.groundReady and self.measures.ground==0:
					self.setVisitingLed(True)
					print self.robName + " visited target area"

				else:
					(lPow, rPow) = self.determineAction("run")
					self.driveMotors(lPow, rPow)

			if (state == 'return'):
				(lPow, rPow) = self.determineAction("return")
				self.driveMotors(lPow, rPow)


	def determineAction(self, state):

		if(self.measures.gpsReady != True or self.measures.compassReady != True):
			return (0,0)

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

		if (center > 4 or right > 4 or left > 4 or collision):
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
			rPow = 0.1
			self.avoid = True
		elif left > 1.5:
			lPow = 0.1
			rPow = 0.0
			self.avoid = True
		else:
			if (state == 'run'):			
				if(self.avoid):
						self.path[:0] = [(self.measures.x, self.measures.y)]
						self.avoid = False
					
				if(self.inicio_parede != None and verificaMesmoPonto(self.inicio_parede,self.measures.x, self.measures.y)):
					self.parede = 'Nenhuma'
					self.inicio_parede = None
					if(right > left):
						lPow = 0.0
						rPow = 0.1
					else:
						lPow = 0.1
						rPow = 0.1
				
				if (beaconReady and beaconVisible and beaconDir > 20.0):
					lPow = 0.0
					rPow = 0.1

				elif (beaconReady and beaconVisible and beaconDir < -20.0):
					lPow = 0.1
					rPow = 0.0

				else:
					lPow = 0
					rPow = 0
					
					if(self.parede == 'Nenhuma' and center < 1.5 and left < 1 and right < 1):						
						lPow = 0.1
						rPow = 0.1
					elif(self.parede == 'Nenhuma' and center < 1.5 and left > 1 and right < 1):
						self.parede = 'Esquerda'
						self.inicio_parede = (self.measures.x, self.measures.y)
						lPow = 0.1
						rPow =0.0
					elif(self.parede == 'Nenhuma' and center < 1.5 and left < 1 and right > 1):
						self.parede = 'Direita'
						self.inicio_parede = (self.measures.x, self.measures.y)
						lPow = 0.0
						rPow =0.1
					elif(self.parede == 'Nenhuma' and center > 1.5):
						if(right >= left):
							self.parede = 'Esquerda'
							self.inicio_parede = (self.measures.x, self.measures.y)
							lPow = 0.0
							rPow = 0.1
						elif(left > right):
							self.parede = 'Direita'
							self.inicio_parede = (self.measures.x, self.measures.y)
							lPow = 0.1
							rPow = 0.0
					elif(self.parede == 'Direita'):
						if (center > 1):
							lPow = 0.1
							rPow = 0.0
						elif(right < 1.5):
							lPow = 0.0
							rPow = 0.1
						else:
							lPow = 0.1
							rPow = 0.1
					elif(self.parede == 'Esquerda'):
						if (center > 1):
							lPow = 0.0
							rPow = 0.1
						elif(left < 1.5):
							lPow = 0.1
							rPow = 0.0
						else:
							lPow = 0.1
							rPow = 0.1
			#Para voltar para tras
			elif (state == 'return'):		
				if self.measures.gpsReady and self.measures.compassReady:
					if(distance((self.measures.x - self.path[0][0]), (self.measures.y - self.path[0][1])) < 1):
						self.path = self.path[1:]		
					compass = self.measures.compass	
					if(self.path == []):
						if distance(self.measures.x - self.startPos[0], self.measures.y - self.startPos[1]) < 0.5:
							self.finish()
							lPow = 0.0
							rPow = 0.0
						else:
							(lPow, rPow) = decision(compass, self.measures.x, self.startPos[0], self.measures.y, self.startPos[1])
						self.path = [(self.startPos[0],self.startPos[1])]
					else:
						if (distance(self.measures.x - self.startPos[0], self.measures.y - self.startPos[1]) < \
							distance((self.measures.x - self.path[0][0]), (self.measures.y - self.path[0][1]))):
								(lPow, rPow) = decision(compass, self.measures.x, self.startPos[0], self.measures.y, self.startPos[1])
								self.path = [(self.startPos[0],self.startPos[1])]
						else:
							(lPow, rPow) = decision(compass, self.measures.x, self.path[0][0], self.measures.y, self.path[0][1])
					
		self.counter += 1
		return (lPow, rPow)	

def verificaMesmoPonto(ponto,x,y):	
	if distance(ponto[0]-x,ponto[1]-y) < 0.25:
		return True
		
	return False
	
def distance(x, y):
	return float((x**2 + y**2)**(float(1)/2))		
	
def decision(compass, x1, x2, y1, y2):	
	dx = x2 - x1
	dy = y2 - y1
	
	abs_target_dir = math.atan2(dy,dx)*180/3.141592
	target_dir = abs_target_dir - compass
	
	if target_dir > 180:
		target_dir -= 360
	elif target_dir < -180:
		target_dir += 360
		
	if target_dir > 15:
		lPow = 0.0
		rPow = 0.1
	elif target_dir < -15:
		lPow = 0.1
		rPow = 0.0
	else:
		lPow = 0.1
		rPow = 0.1
	return (lPow,rPow)	
		
rob = MyRob("AA",3,"localhost")
rob.run()
