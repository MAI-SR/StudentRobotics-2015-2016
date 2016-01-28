#########################################################################################################################################
#Disclaimer:    																														#
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
#Constants
	#Hard
wheelCircumfrence = 0.314
robotCircumfrence = 1.1

tpcm = 101.86 #ticks per centimeter (motor) [genauer: 101,85916357881301489208560855841]
	#Soft
motorspeed = 50
turnpwer = 50

ServoGrabOpen = 70 #ToDo
ServoGrabClose = 20 #ToDo
ServoCamUp = 50 #ToDo
ServoCamDown = 30 #ToDo
ServoLiftUp = 30 #ToDo
ServoLiftDown = 30 #ToDo
ServoTiltUp = 30 #ToDo
ServoTiltDown = 30 #ToDo


abortSearchTime = 100 #if searching and not enough time is left
remainingTime = 180
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
#End Threads API
#Variables
state = "start"
currentFrontToken = None

currentFL = 0
currentBL = 0
currentFR = 0
currentBR = 0
#End Variables 
#Methods

#Setter
def resetcurrent():
	currentFL = 0
	currentBL = 0
	currentFR = 0
	currentBR = 0

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
		
def setServo(servo, value = 0): #Servos for grabbing the token
	if servo == "ArmF": #ServoArmFront
		R.servos[0][0] = value
	elif servo == "ArmL": #ServoArmLeft
		R.servos[0][1] = value
	elif servo == "ArmB": #ServoArmBack
		R.servos[0][2] = value
	elif servo == "ArmR": #ServoArmRight
		R.servos[0][3] = value
	elif servo == "TokenTilt": #ServoTokenTilt
		R.servos[0][4] = value
	elif servo == "TokenLift": #ServoTokenLift
		R.servos[0][5] = value
	elif servo == "Cam": #ServoCam
		R.servos[0][6] = value
#End Setter
#Getter
#End Getter
def drive_F_B(distance): #forward & backward
	distance /= 2#every motor only has to drive half the total distance due to vectoradition
	distance *= math.sqrt(2)#length of the vector every motor has to drive
	tickdistance = int(abs((distance/wheelCircumfrence)*3200))#ticks every motor has to drive
	tickcount = 0#amount of ticks alredy driven
	if distance > 0:#forward
		setAllMotor(-1, -1, 1, 1)
		direction = 1
	else:#backward
		setAllMotor(1, 1, -1, -1)
		direction = 0
	while tickdistance > tickcount:
		tickcount += motorSpeedCoretion_F_B_and_L_R()
	setAllMotor(0, 0, 0, 0)
	resetcurrent()

	
def drive_L_R(distance): #left & right
	distance /= 2#every motor only has to drive half the total distance due to vectoradition
	distance *= math.sqrt(2)#length of the vector every motor has to drive
	tickdistance = int(abs((distance/wheelCircumfrence)*3200))#ticks every motor has to drive
	tickcount = 0#amount of ticks alredy driven
	if distance > 0:#right
		setAllMotor(-1, 1, -1, 1)
		direction = 1
	else:#left
		setAllMotor(1, -1, 1, -1)
		direction = 0
	while tickdistance > tickcount:
		tickcount += motorSpeedCoretion_F_B_and_L_R()
	setAllMotor(0, 0, 0, 0)
	resetcurrent()

def motorSpeedCoretion_F_B_and_L_R():
    global currentFL, currentBL, currentFR, currentBR
    tick_FL = R.ruggeduinos[0].motorStatusFL()
    tick_BL = R.ruggeduinos[0].motorStatusBL()
    tick_FR = R.ruggeduinos[0].motorStatusFR()
    tick_BR = R.ruggeduinos[0].motorStatusBR()
    currentFL += tick_FL
    currentBL += tick_BL
    currentFR += tick_FR
    currentBR += tick_BR
    average = (currentFL + currentBL + currentFR + currentBR)/4.0
    if average >= 100:
    	setMotor("FL", R.motors[1].m1.power*(average/currentFL))
    	setMotor("BL", R.motors[1].m0.power*(average/currentBL))
    	setMotor("FR", R.motors[0].m0.power*(average/currentFR))
    	setMotor("BR", R.motors[0].m1.power*(average/currentBR))
    	resetcurrent()
    return int((tick_FL + tick_BL + tick_FR + tick_BR)/4)

def drive_FR_BL(distance): #forward right & backward left
	print "ToDo"
	
def drive_FL_BR(distance): #forward left & backward right
	print "ToDo"

def dist_to_ticks_diagonal(cm):
	return cm*tpcm

def dist_to_ticks_straight(cm):
	return (cm/math.sqrt(2))*tpcm
	
def turn(degree): #turn
    tickcount = 0
    degreeticks = int(((abs(degree/360)*robotCircumfrence)/wheelCircumfrence)*3200)
    if degree > 0:#+ = Counterclockwise
        setAllMotor(1, 1, 1, 1)
    else:#- = Clockwise
        setAllMotor(-1, -1, -1, -1)
    while degreeticks > tickcount:
        tickcount += int((motorStatusFL()+motorStatusBL()+motorStatusFR()+motorStatusBR())/4)
    setAllMotor(0, 0, 0, 0)

def search():
	if (remainingTime <= abortSearchTime and (hasTokenF or hasTokenL or hasTokenB or hasTokenR)) or (hasTokenF and hasTokenL and hasTokenB and hasTokenR): #if (not enough time left and has at least one token) or (has already all tokens)
		state = "calcPos"
		crossStateInfo = None
		return
	counter = 0
	found = False
	while counter < 24 or not found:
		found = False
		markers = R.see()
		if len(markers) == 0:
			turn(15)
		else:
			markers = markers.sort()
			for m in markers:
				if m.info.marker_type == MARKER_TOKEN_TOP or m.info.marker_type == MARKER_TOKEN_SIDE or m.info.marker_type == MARKER_TOKEN_BOTTOM:
					found = True
					crossStateInfo = m
					break				
		if found:
			state = "gotoToken"
			return
		else:
			turn(15)

def gotoToken(m):
	turn(m.polar.x)
	drive_F_B(m.dist)

turnDict = {
	'z0id32':0, 'z0id33':0, 'z0id34':5, 'z0id35':1, 'z0id36':4, 'z0id37':3, 'z0id38':0, 'z0id39':0, 'z0id40':5, 'z0id41':4, 'z0id42':1, 'z0id43':3, 'z0id44':0, 'z0id45':0, 'z0id46':5, 'z0id47':3, 'z0id48':1, 'z0id49':4,
	'z1id32':1, 'z1id33':3, 'z1id34':3, 'z1id35':5, 'z1id36':1, 'z1id37':4, 'z1id38':2, 'z1id39':2, 'z1id40':4, 'z1id41':5, 'z1id42':3, 'z1id43':1, 'z1id44':3, 'z1id45':1, 'z1id46':1, 'z1id47':5, 'z1id48':4, 'z1id49':3,
	'z2id32':2, 'z2id33':2, 'z2id34':4, 'z2id35':3, 'z2id36':5, 'z2id37':1, 'z2id38':1, 'z2id39':3, 'z2id40':3, 'z2id41':1, 'z2id42':5, 'z2id43':4, 'z2id44':1, 'z2id45':3, 'z2id46':3, 'z2id47':4, 'z2id48':5, 'z2id49':1,
	'z3id32':3, 'z3id33':1, 'z3id34':1, 'z3id35':4, 'z3id36':3, 'z3id37':5, 'z3id38':3, 'z3id39':1, 'z3id40':1, 'z3id41':3, 'z3id42':4, 'z3id43':5, 'z3id44':2, 'z3id45':2, 'z3id46':4, 'z3id47':1, 'z3id48':3, 'z3id49':5
	}

def turnToken():
	m = currentFrontToken
	dictentry = 'z{0}id{1}'.format(R.zone, m.info.code)
	orientat = 0
	baseCase = turnDict[dictentry]
	case = 0
	
	if m.orientation.rot_y < 5 or m.orientation.rot_y > 355:#Orientation F
		orientat += 0
	elif m.orientation.rot_y < 265 or m.orientation.rot_y > 275:#Orientation R
		orientat += 1
	elif m.orientation.rot_y < 85 or m.orientation.rot_y > 355:#Orientation L
		orientat += 2
	elif m.orientation.rot_y < 175 or m.orientation.rot_y < 185:#Orientation B
		orientat += 3
	
	if baseCase == 0 or baseCase == 1 or baseCase == 2 or baseCase == 3:
		case = (baseCase + orientat) % 4
	elif baseCase == 4 or baseCase == 5:6
		case = baseCase
		
	if case == 0:#A - R'/R3
	elif case == 1:#C - U R
	elif case == 2:#D - R
	elif case == 3:#B - U' R
	elif case == 4:#E - R2
	elif case == 5:#F - /


def rps(m):#@Parameter ArenaMarker @return (x,y,rot)
	distToWall = m.world.z
	distToLeftWall = ((m.info.code % 7) + 1) + m.world.x
	relrot = m.orientation.rot_z
	
	if m.info.code >= 0 and m.info.code <= 6:
		if R.zone == 0:
			return (distToWall, distToLeftWall, 180 + relrot)
		if R.zone == 1:
			return (8-distToLeftWall, distToWall, -90 + relrot)
		if R.zone == 2:
			return (8-distToWall, 8-distToLeftWall, 0 + relrot)
		if R.zone == 3:
			return (distToLeftWall, 8-distToWall, 90 + relrot)
	if m.info.code >= 7 and m.info.code <= 13:
		if R.zone == 0:
			return (distToLeftWall, 8-distToWall, 90 + relrot)
		if R.zone == 1:
			return (distToWall, distToLeftWall, 180 + relrot)
		if R.zone == 2:
			return (8-distToLeftWall, distToWall, -90 + relrot)
		if R.zone == 3:
			return (8-distToWall, 8-distToLeftWall, 0 + relrot)
	if m.info.code >= 14 and m.info.code <= 20:
		if R.zone == 0:
			return (8-distToWall, 8-distToLeftWall, 0 + relrot)
		if R.zone == 1:
			return (distToLeftWall, 8-distToWall, 90 + relrot)
		if R.zone == 2:
			return (distToWall, distToLeftWall, 180 + relrot)
		if R.zone == 3:
			return (8-distToLeftWall, distToWall, -90 + relrot)
	if m.info.code >= 21 and m.info.code <= 27:
		if R.zone == 0:
			return (8-distToLeftWall, distToWall, -90 + relrot)
		if R.zone == 1:
			return (8-distToWall, 8-distToLeftWall, 0 + relrot)
		if R.zone == 2:
			return (distToLeftWall, 8-distToWall, 90 + relrot)
		if R.zone == 3:
			return (distToWall, distToLeftWall, 180 + relrot)
	
		
#End Methods
#Classes
class ArenaPoint(): #Class ArenaPoint used for calculating the robots place in the arena
	def __init__(self, x, y, rot, dist):
		self.x = x #x value of a point inside the arena
		self.y = y #y value of a point inside the arena
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
			errorTokenF = readUSF() >= 10 and hasTokenF
			errorTokenL = readUSL() >= 10 and hasTokenL
			errorTokenB = readUSB() >= 10 and hasTokenB
			errorTokenR = readUSR() >= 10 and hasTokenR
			errorToken = errorTokenF or errorTokenL or errorTokenB or errorTokenR
#End Classes
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
		turnToken()
	
	elif state == "putdownToken":
		print "ToDo"
	
	elif state == "switchToken":
		print "ToDo"
	
	elif state == "stop":
		print "ToDo"
	
	else:
		print "You broke it! L3rn 70 wr1t3 y0u f4g!"
#End Main Method