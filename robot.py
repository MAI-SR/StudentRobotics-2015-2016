#########################################################################################################################################
#Disclaimer:																															#
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
        return int(resp)
		
    def motorStatusBL(self):
        with self.lock:
            resp=self.command("x")
        return int(resp)
		
    def motorStatusFR(self):
        with self.lock:
            resp=self.command("y")
        return int(resp)
		
    def motorStatusBR(self):
        with self.lock:
            resp=self.command("z")
        return int(resp)
R = Robot.setup()
R.ruggeduino_set_handler_by_fwver("SRcustom", CustomisedRuggeduino)
R.init()
R.wait_start()
#End Custom Ruggeduino
#Constants
	#Hard
wheelCircumfrence = 0.314
robotCircumfrence = 1.1

tpcm = 101.86 #ticks per centimeter (motor) [genauer: 101,85916357881301489208560855841]
	#Soft
motorspeed = 100
turnpwer = 100 #motor power while turning

abortSearchTime = 100 #if searching and not enough time is left
#End Constants
#Threads API
	#error
errorToken = False #lost a token?
errorTokenF = False #lost the token?
errorTokenL = False #lost the token?
errorTokenB = False #lost the token?
errorTokenR = False #lost the token?
	#SensorThread
hasTokenF = False #should there be a Token?
hasTokenL = False #should there be a Token?
hasTokenB = False #should there be a Token?
hasTokenR = False #should there be a Token?
	#TimeThread
remainingTime = 180
#End Threads API
#Variables
state = "start"
#End Variables 
#Methods

#Setter
def setMotor(motor, speed = 0): #set Motor power
	if motor == "FL": #front left
		R.motors[0].m0.power = speed
	elif motor == "BL": #back left
		R.motors[0].m1.power = speed
	elif motor == "FR": #front right
		R.motors[1].m0.power = speed
	elif motor == "BR": #back right
		R.motors[1].m1.power = speed
		
def setAllMotor(fl, bl, fr, br): #set all motors
	setMotor("FL", fl*motorspeed)
	setMotor("BL", bl*motorspeed)
	setMotor("FR", fr*motorspeed)
	setMotor("BR", br*motorspeed)
		
def setServo(servo, value = 0): #set Servo position
	if servo == "": #Servo0Name
		R.servos[0][0] = value
	elif servo == "": #Servo1Name
		R.servos[0][1] = value
	elif servo == "": #Servo2Name
		R.servos[0][2] = value
	elif servo == "": #Servo3Name
		R.servos[0][3] = value
#End Setter
#Getter
#End Getter
def drive_F_B(distance): #forward & backward
	if distance > 0:
		setAllMotor(-1, -1, 1, 1)
	else:
		setAllMotor(1, 1, -1, -1)
	drive = DriveThread(2, "drive", 2, dist_to_ticks_straight(distance))
	drive.start()
	drive.join()
	setAllMotor(0, 0, 0, 0)
	
def drive_L_R(distance): #left & right
	print "ToDo"

def drive_FR_BL(distance): #forward right & backward left
	print "ToDo"
	
def drive_FL_BR(distance): #forward left & backward right
	print "ToDo"

def dist_to_ticks_diagonal(cm):
	return cm*tpcm

def dist_to_ticks_straight(cm):
	return (cm/sqrt(2))*tpcm
	
def turn(degree): #turn 
	print "ToDo"
	if degree >= 0:
		setMotor("FL", turnpower)
		setMotor("BL", turnpower)
		setMotor("FR", turnpower)
		setMotor("BR", turnpower)
	else:
		setMotor("FL", -turnpower)
		setMotor("BL", -turnpower)
		setMotor("FR", -turnpower)
		setMotor("BR", -turnpower)
	tickdegree = abs((((degree / 360.0) * robotCircumfrence) / wheelCircumfrence) * 3200)
	tickcount = 0
	while tickcount < tickdegree:
		tickcount = tickcount + abs(motorStatusFL)
	setMotor("FL", 0)
	setMotor("BL", 0)
	setMotor("FR", 0)
	setMotor("BR", 0)

def search():
	if (remainingTime <= abortSearchTime && (hasTokenF || hasTokenL || hasTokenB || hasTokenR)) || (hasTokenF && hasTokenL && hasTokenB && hasTokenR): #if (not enough time left and has at least one token) or (has already all tokens)
		state = "calcPos"
		crossStateInfo = None
		return
	counter = 0
	found = False
	while counter < 24 || !found:
		markers = R.see()
		if len(markers) == 0:
			turn(15)
		else:
			markers = markers.sort()
			for m in markers:
				if m.info.marker_type == MARKER_TOKEN_TOP || m.info.marker_type == MARKER_TOKEN_SIDE || m.info.marker_type == MARKER_TOKEN_BOTTOM
					found = True
					crossStateInfo = m
					break
				else:
					turn(15)
	if found:
		state = "gotoToken"

def gotoToken(m):
	turn(m.polar.x)
	drive(m.dist)
	
#End Methods
#Main Method
	#Thread start
timeT = TimeThread(1, "time", 1)
timeT.start()
sensorsT = SensorThread(2, "sensors", 2)
sensorsT.start()
	#End Thread start
crossStateInfo = None
while True:
	if state == "start":
		state = "searching"
	
	elif state == "searching":
		search()
		
	elif state == "gotoToken":
		gotoToken(crossStateInfo)
		print "ToDo"
	
	elif state == "pickupToken":
		print "ToDo"
	
	elif state == "calcPos":
		print "ToDo"
		state = "gotoCorner"
	
	elif state == "gotoCorner":
		print "ToDo"
		state = "turnToken"
	
	elif state == "turnToken":
		print "ToDo"
	
	elif state == "putdownToken":
		print "ToDo"
	
	elif state == "switchToken":
		print "ToDo"
	
	elif state == "stop":
		print "ToDo"
	
	else:
		print "You broke it! L3rn 70 wr1t3 y0u f4g!"
#End Main Method
#Classes
class Point(): #Class Point used for calculating the robots place in the arena
	def __init__(x,y,rot,dist):
		self.x = x #x value of the point inside the arena
		self.y = y #y value of the point inside the arena
class TimeThread (threading.Thread):
	def __init__(self, threadID, name, counter):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.counter = counter
	def run(self):
		endTime = time.time() + 180
		while True:
			remainingTime = endTime - time.time()
class SensorThread (threading.Thread):
	def __init__(self, threadID, name, counter):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.counter = counter
	def run(self):
		while True:
			errorTokenF = readUSF >= 10 && hasTokenF
			errorTokenL = readUSL >= 10 && hasTokenL
			errorTokenB = readUSB >= 10 && hasTokenB
			errorTokenR = readUSR >= 10 && hasTokenR
			errorToken = errorTokenF || errorTokenL || errorTokenB || errorTokenR
class DriveThread (threading.Thread):
	def __init__(self, threadID, name, counter, ticks):
		threading.Thread.__init__(self)
		self.threadID = threadID
		self.name = name
		self.counter = counter
		self.ticks = ticks
	def run(self):
		while True:
#End Classes