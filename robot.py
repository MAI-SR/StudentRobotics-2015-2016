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
R = Robot.setup()
R.ruggeduino_set_handler_by_fwver("SRcustom", CustomisedRuggeduino)
R.init()
R.wait_start()
#End Custom Ruggeduino
#Constants

#End Constants
#Variables
robotrotation = 45#current direction the robot is facing
robotX = 0#current x position of the robot
robotY = 0#current y position of the robot
calculatedUS#value the ultasonic sensor is suposed to give clockwhise
token = null#current token the robot is trying to get or null for when its not searcing for a token
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

def calculateUS(usnumber):#calculates the value the ulrasonic sensor is suposed to give
	rotation = robotrotation + usnumber*90#magic
	if rotation == token.rotation:
		calculatedUS = token
		return
	barrier = false
	dist = 0.00
	while barrier == false:
		dist = dist+0.05
		tmpPoint = calculatePoint(dist,rotation)
		barrier = inBarrier(tmpPoint[0],tmpPoint[1])
	calculatedUS = Point(tmpPoint[0],tmpPoint[1],rotation,dist)
	
def alculatePoint(dist, rotation):#calculates a point in relation to the robots current position
	tmpX = robotX + math.cos(rotation)*dist#magic
	tmpY = robotY + math.sin(rotation)*dist
	return (tmpX, tmpY)

def inBarrier(x, y):#calculates if a point is inside a barrier
	if():#magic

def drive_F_B(distance): #forward & backward
	
def drive_FL_BR(distance): #forward left & backward right

def drive_L_R(distance): #left & right

def drive_FR_BL(distance): #forward right & backward left
	
def turn(degree):

#End Methods
#Main Method

#End Main Method
#Classes
class Point():#Class Point used for calculating the robots place in the arena
	def __init__(x,y,rot,dist):
		self.x = x#x value of the point inside the arena
		self.y = y#y value of the point inside the as
		self.rotation = rot#direction the robot has to face to face the point
		self.distance = dist#distance between robot and point
#End Classes