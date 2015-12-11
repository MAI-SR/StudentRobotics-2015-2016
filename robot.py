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
#End Threads API
#Variables

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

#End Methods
#Main Method
	#Thread start
sensors = SensorThread(1, "sensors", 1)
sensors.start()
	#End Thread start

#End Main Method
#Classes
class Point(): #Class Point used for calculating the robots place in the arena
	def __init__(x,y,rot,dist):
		self.x = x #x value of the point inside the arena
		self.y = y #y value of the point inside the arena
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