#########################################################################################################################################
#Disclaimer:        																													#
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
wheelCircumfrence = 0.3141
robotCircumfrence = 1.0838#.....

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

#End Variables 
#Methods

#Setter
def setMotor(motor, speed): #set Motor power
    if motor == "FL": #front left
		R.motors[1].m1.power = int(speed)
    elif motor == "BL": #back left
    	R.motors[1].m0.power = int(speed)
    elif motor == "FR": #front right
    	R.motors[0].m0.power = int(speed)
    elif motor == "BR": #back right
    	R.motors[0].m1.power = int(speed)

def setAllMotor(fl, bl, fr, br): #set all motors
    setMotor("FL", fl)
    setMotor("FR", fr)
    setMotor("BR", br)
    setMotor("BL", bl)

#End Setter
#Getter
#End Getter
#Move Arms
def grabSide(side, state):#true == zu
    if(side == 'right'):
        if(state):
            R.servos[0][1] = -10
        else:
            R.servos[0][1] = 70
    elif(side == 'left'):
        if(state):
            R.servos[0][5] = 10
        else:
            R.servos[0][5] = -70
    elif(side == 'back'):
        if(state):
            R.servos[0][0] = -10
        else:
            R.servos[0][0] = 70
    elif(side == 'front'):
        if(state):
            R.servos[0][7] = -40
        else:
            R.servos[0][7] = 60
    else:
        print 'You missspelled back/left/right/front, by spelling' + str(state)

def moveArm(there):
    if(there == 'up'):
        R.servos[0][4] = -35
    elif(there == 'down'):
        R.servos[0][4] = -85
    elif(there == 'middle'):
        R.servos[0][4] = -60
    else:
        print 'You missspelled up/down/middle, by spelling' + str(there)
#End Move Arms
#Movement
def drive_F_B(distance):#distance in meters
    distance /= 2.0#every motor only has to drive half the total distance due to vectoradition
    distance *= math.sqrt(2)#length of the vector every motor has to drive
    tickdistance = int(abs((distance/wheelCircumfrence)*3200))#ticks every motor has to drive
    tickcount = 0#amount of ticks alredy driven
    if distance > 0:#forward
        setAllMotor(-47, -47, 50, 50)
        direction = 1
    else:#backward
        setAllMotor(47, 47, -50, -50)
        direction = 0
    print tickdistance
    while tickdistance > tickcount:
        tickcount += int((R.ruggeduinos[0].motorStatusFL()+R.ruggeduinos[0].motorStatusFR())/2)
    setAllMotor(0, 0, 0, 0)

def drive_L_R(distance):
    distance /= 2.0#every motor only has to drive half the total distance due to vectoradition
    distance *= math.sqrt(2)#length of the vector every motor has to drive
    tickdistance = int(abs((distance/wheelCircumfrence)*3200))#ticks every motor has to drive
    tickcount = 0#amount of ticks alredy driven
    if distance > 0:#right
        setAllMotor(-47, 50, -47, 50)
    	direction = 1
    else:#left
    	setAllMotor(47, -50, 47, -50)
    	direction = 0
    print tickdistance
    while tickdistance > tickcount:
        tickcount += int((R.ruggeduinos[0].motorStatusFL()+R.ruggeduinos[0].motorStatusBL())/2)
    setAllMotor(0, 0, 0, 0)

def turn(degree): #turn
    tickcount = 0
    if degree > 0:#+ = Counterclockwise
        setAllMotor(40, 40, 40, 40)
    else:#- = Clockwise
    	setAllMotor(-40, -40, -40, -40)
    degreeticks = int(((abs(degree/360.0)*robotCircumfrence)/wheelCircumfrence)*3200)
    print 'test'
    while degreeticks > tickcount:
        print 'loop'
    	tickcount += R.ruggeduinos[0].motorStatusFL()
        print tickcount
    setAllMotor(0, 0, 0, 0)
#End Movement
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

#Turn Token
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
    elif baseCase == 4 or baseCase == 5:
        case = baseCase
    	
    if case == 0:#A - R'/R3
        turnTokenPart(3)
    elif case == 1:#C - U R
        regrab('U')
        turnTokenPart(1)
    elif case == 2:#D - R
        turnTokenPart(1)
    elif case == 3:#B - U' R
        regrab('V')
        turnTokenPart(1)
    elif case == 4:#E - R2
        turnTokenPart(2)
    #elif case == 5:#F - /

def turnTokenUp():#R #we are asuming that the robot is holding the token in question or standing right in front of it
    grabSide('front', False)
    time.sleep(0.5)
    moveArm('down')
    time.sleep(1)
    grabSide('front', True)
    time.sleep(1)
    moveArm('up')
    time.sleep(2)
    grabSide('front', False)
    time.sleep(0.5)
    moveArm('middle')
    time.sleep(1)
    grabSide('front', True)
    time.sleep(0.5)

def turnTokenPart(steps):#in steps
    if(steps == 1):
        turnTokenUp()
    elif(steps == 2):
        turnTokenUp()
        turnTokenUp()
    elif(steps == 3):
        turnTokenUp()
        turnTokenUp()
        turnTokenUp()
    else:
        print 'A your dumb, or B your finger sliped'

def regrab(direction):#U = right, V = left
    grabSide('front', False)
    time.sleep(0.4)
    drive_F_B(-0.3)
    if direction == 'U':
        drive_L_R(-0.45)
        turn(90)
        drive_L_R(-0.425)
        drive_F_B(0.45)
    else:
        drive_L_R(0.45)
        turn(-90)
        drive_L_R(0.425)
        drive_F_B(-0.45)
    
#End Turn Token

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
			errorTokenF = R.ruggeduinos[0].readUSF() >= 10 and hasTokenF
			errorTokenL = R.ruggeduinos[0].readUSL() >= 10 and hasTokenL
			errorTokenB = R.ruggeduinos[0].readUSB() >= 10 and hasTokenB
			errorTokenR = R.ruggeduinos[0].readUSR() >= 10 and hasTokenR
			errorToken = errorTokenF or errorTokenL or errorTokenB or errorTokenR
#End Classes
#Main Method
print 'Hello, world'
drive_L_R(2)