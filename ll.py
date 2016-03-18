#########################################################################################################################################
#Disclaimer:                        																									#
#---------------------------------------------------------------------------------------------------------------------------------------#
#You can use this code for any and all educational purposes.																			#
#If you want to use it for the Student Robotics Contest you will have to send us an email and link our GitHub repository in your code.	#
#Also we would like you to mention us and/or Team MAI when you talk about your code to others.											#
#Our email is: mai.studentrobotics@gmail.com																							#
#########################################################################################################################################

from sr.robot import *
from decimal import *
import threading
import time
import math

#ALL VALUES IN SAME UNITS: meter, degree, 

#Constants
ticksPerMeter = 3200#number of ticks per meter for the motors
wheelCircumference = 0.3141#circumference of a single wheel
robotCircumference = 1.0838#circumference of the robot


motorSpeedFBFastFL = -47
motorSpeedFBFastBL = -47
motorSpeedFBFastFR = 50
motorSpeedFBFastBR = 50

motorSpeedFBSlowFL = -23
motorSpeedFBSlowBL = -23
motorSpeedFBSlowFR = 25
motorSpeedFBSlowBR = 25

motorSpeedLRFastFL = -47
motorSpeedLRFastBL = 50
motorSpeedLRFastFR = -47
motorSpeedLRFastBR = 50

motorSpeedLRSlowFL = -23
motorSpeedLRSlowBL = 25
motorSpeedLRSlowFR = -23
motorSpeedLRSlowBR = 25

motorSpeedTurnFL = 25
motorSpeedTurnBL = 25
motorSpeedTurnFR = 25
motorSpeedTurnBR = 25


t = Time(0, "time", 0)
#End Constants
#Variables
#End Variables
#Custom Ruggeduino
class CustomisedRuggeduino(Ruggeduino):
    def readUSF(self):
        with self.lock:
    		resp=self.command("b")
        return int(resp)
        
    def readUSL(self):
        with self.lock:
            resp=self.command("c")
        return int(resp)
		
    def readUSB(self):
        with self.lock:
            resp=self.command("d")
        return int(resp)
		
    def readUSR(self):
        with self.lock:
            resp=self.command("e")
        return int(resp)
	
	def motorReset(self):
		with self.lock:
			resp=self.command("u")
		return int(resp)
		
    def motorStatusFL(self):
        with self.lock:
            resp=self.command("w")
        return abs(int(resp))
		
    def motorStatusBL(self):
        with self.lock:
            resp=self.command("x")
        return abs(int(resp))
		
    def motorStatusFR(self):
        with self.lock:
            resp=self.command("y")
        return abs(int(resp))
		
    def motorStatusBR(self):
        with self.lock:
            resp=self.command("z")
        return abs(int(resp))
R = Robot.setup()
R.ruggeduino_set_handler_by_fwver("SRcustom", CustomisedRuggeduino)
R.init()
R.wait_start()
#End Custom Ruggeduino

#Motors
def setMotor(fl = None, bl = None, fr = None, br = None):
	print 'setMotor'
	if not fl == None:
		R.motors[1].m1.power = int(fl)
	if not bl == None:
		R.motors[1].m0.power = int(bl)
	if not fr == None:
		R.motors[0].m0.power = int(fr)
	if not br == None:
		R.motors[0].m1.power = int(br)
#Drive
def llDriveFB(dist):#positive: front
	distCorrect = (dist/2.0) * math.sqrt(2)#distance the wheels actually have to drive
	distTick = (abs(distCorrect)/wheelCircumference)*ticksPerMeter#distance in ticks
	print 'distTick: ' + str(distTick)
	currentDistTick = 0
	if dist > 1:
		setMotor(fl = motorSpeedFBSlowFL, bl = motorSpeedFBSlowBL, fr = motorSpeedFBSlowFR, br = motorSpeedFBFastBR)
		time.sleep(0.5)
		setMotor(fl = motorSpeedFBFastFL, bl = motorSpeedFBFastBL, fr = motorSpeedFBFastFR, br = motorSpeedFBFastBR)
	elif dist > 0:
		setMotor(fl = motorSpeedFBSlowFL, bl = motorSpeedFBSlowBL, fr = motorSpeedFBSlowFR, br = motorSpeedFBFastBR)
	elif dist < -1:
		setMotor(fl = -motorSpeedFBSlowFL, bl = -motorSpeedFBSlowBL, fr = -motorSpeedFBSlowFR, br = -motorSpeedFBFastBR)
		time.sleep(0.5)
		setMotor(fl = -motorSpeedFBFastFL, bl = -motorSpeedFBFastBL, fr = -motorSpeedFBFastFR, br = -motorSpeedFBFastBR)
	elif dist <= 0:
		setMotor(fl = -motorSpeedFBSlowFL, bl = -motorSpeedFBSlowBL, fr = -motorSpeedFBSlowFR, br = -motorSpeedFBFastBR)
	else:
		raise TypeError()
	while currentDistTick < distTick:
		currentDistTick += (getTicks('FL') + getTicks('FR'))/2
		if (distTick - currentDistTick) <  3200:
			if dist > 0:
				setMotor(fl = motorSpeedFBSlowFL, bl = motorSpeedFBSlowBL, fr = motorSpeedFBSlowFR, br = motorSpeedFBFastBR)
			else:
				setMotor(fl = -motorSpeedFBSlowFL, bl = -motorSpeedFBSlowBL, fr = -motorSpeedFBSlowFR, br = -motorSpeedFBFastBR)
	setMotor(fl = 0, bl = 0, fr = 0, br = 0)

def llDriveLR(dist):#positive: left
	distCorrect = (dist/2.0) * math.sqrt(2)#distance the wheels actually have to drive
	distTick = (abs(distCorrect)/wheelCircumference)*ticksPerMeter#distance in ticks
	print 'distTick: ' + str(distTick)
	currentDistTick = 0
	if dist > 1:
		setMotor(fl = motorSpeedLRSlowFL, bl = motorSpeedLRSlowBL, fr = motorSpeedLRSlowFR, br = motorSpeedLRFastBR)
		time.sleep(0.5)
		setMotor(fl = motorSpeedLRFastFL, bl = motorSpeedLRFastBL, fr = motorSpeedLRFastFR, br = motorSpeedLRFastBR)
	elif dist > 0:
		setMotor(fl = motorSpeedLRSlowFL, bl = motorSpeedLRSlowBL, fr = motorSpeedLRSlowFR, br = motorSpeedLRFastBR)
	elif dist < -1:
		setMotor(fl = -motorSpeedLRSlowFL, bl = -motorSpeedLRSlowBL, fr = -motorSpeedLRSlowFR, br = -motorSpeedLRFastBR)
		time.sleep(0.5)
		setMotor(fl = -motorSpeedLRFastFL, bl = -motorSpeedLRFastBL, fr = -motorSpeedLRFastFR, br = -motorSpeedLRFastBR)
	elif dist <= 0:
		setMotor(fl = -motorSpeedLRSlowFL, bl = -motorSpeedLRSlowBL, fr = -motorSpeedLRSlowFR, br = -motorSpeedLRFastBR)
	else:
		raise TypeError()
	while currentDistTick < distTick:
		currentDistTick += (getTicks('FL') + getTicks('FR'))/2
		if (distTick - currentDistTick) <  3200:
			if dist > 0:
				setMotor(fl = motorSpeedLRSlowFL, bl = motorSpeedLRSlowBL, fr = motorSpeedLRSlowFR, br = motorSpeedLRFastBR)
			else:
				setMotor(fl = -motorSpeedLRSlowFL, bl = -motorSpeedLRSlowBL, fr = -motorSpeedLRSlowFR, br = -motorSpeedLRFastBR)
	setMotor(fl = 0, bl = 0, fr = 0, br = 0)
#End Drive
#Turn
def llTurn(degree):#positive: counter clockwise
	while degree > 180:
        degree -= 360
	while degree < -180:
		degree += 360
	degreeTick = ((abs(degree/360.0)*robotCircumference)/wheelCircumference)*ticksPerMeter
	currentDegreeTick = 0
	if degree > 0:
		setMotor(fl = motorSpeedTurnFL, bl = motorSpeedTurnBL, fr = motorSpeedTurnFR, br = motorSpeedTurnBR)
	else:
		setMotor(fl = -motorSpeedTurnFL, bl = -motorSpeedTurnBL, fr = -motorSpeedTurnFR, br = -motorSpeedTurnBR)
	while currentDegreeTick < degreeTick:
		currentDegreeTick += getTicks('FL')
	setMotor(fl = 0, bl = 0, fr = 0, br = 0)
#End Turn
#End Motors

def llGrab(f = None, r = None, l = None, b = None):#'F'/'R'/'L'/'B' #True: closed
	if not f == None:
		if f:
            R.servos[0][7] = -40
        else:
            R.servos[0][7] = 80
	if not r == None:
		if r:
		    R.servos[0][1] = -10
        else:
            R.servos[0][1] = 70
	if not l == None:
		if l:
            R.servos[0][5] = 10
        else:
            R.servos[0][5] = -70
	if not b == None:
		if b:
            R.servos[0][0] = -10
        else:
            R.servos[0][0] = 70
			
def llArmState(state):#'U'/'M'/'D'
	if state == 'U':
		R.servos[0][4] = -15
	elif state == 'M':
		R.servos[0][4] = -60
	elif state == 'D':
		R.servos[0][4] = -85
	else:
		raise ValueError()

def llCamState(state):#'U'/'D'
	if state == 'U':
		R.servos[0][3] = 50
	elif state == 'D':
		R.servos[0][3] = 10
	else:
		raise ValueError()
#API
def driveFB(dist):#positive: front
	print 'driveFB ' + str(dist)
	return llDriveFB(dist)
def driveLR(dist):#positive: left
	print 'driveLR ' + str(dist)
	return llDriveLR(dist)
def turn(degree):#positive: counterclockwise
	print 'turn ' + str(degree)
	return llTurn(degree)
def grab(arm, state):#'F'/'R'/'L'/'B' #True: closed
	print 'grab ' + arm + str(state)
	return llGrab(arm, state)
def armState(state):#'U'/'M'/'D'
	print 'armState ' + str(state)
	return llArmState(state)
def camState(state):#'U'/'D'
	print 'camState ' + str(state)
	return llCamState(state)
def getRemainingTime():
	print 'getRemainingTime'
	return t.getRemainingTime()
#LibAPI
def zone():
	print 'zone'
	return R.zone

def scan():
	print 'scan'
	return R.see()
	
def getTicks(motor):#'FL'/'BL'/'FR'/'BR'
	print 'getTicks ' + motor
	if motor == 'FL':
		return R.ruggeduino[0].motorStatusFL()
	elif motor == 'BL':
		return R.ruggeduino[0].motorStatusBL()
	elif motor == 'FR':
		return R.ruggeduino[0].motorStatusFR()
	elif motor == 'BR':
		return R.ruggeduino[0].motorStatusBR()
	else:
		raise ValueError()
#End LibAPI
#End API

def setup():
	t.start()

class Time(threading.Thread):
	remainingTime = 180
	def __init__(self, threadID, name, counter):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.counter = counter
	def run(self):
		global remainingTime
		endTime = time.time() + 180
		while True:
			remainingTime = endTime - time.time()
	def getRemainingTime():
		return remainingTime