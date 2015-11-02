from sr.robot import *
from decimal import *
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

#End Constants
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

def setServo(servo, value = 0):
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
	print "ToDo"
	
def drive_FL_BR(distance): #forward left & backward right
	print "ToDo"

def drive_L_R(distance): #left & right
	print "ToDo"

def drive_FR_BL(distance): #forward right & backward left
	print "ToDo"
	
def turn(degree):
	print "ToDo"

#End Methods
#Main Method

#End Main Method
#Classes
class Point(): #Class Point used for calculating the robots place in the arena
	def __init__(x,y,rot,dist):
		self.x = x #x value of the point inside the arena
		self.y = y #y value of the point inside the arena
#End Classes