#########################################################################################################################################
#Disclaimer:                            																								#
#---------------------------------------------------------------------------------------------------------------------------------------#
#You can use this code for any and all educational purposes.																			#
#If you want to use it for the Student Robotics Contest you will have to send us an email and link our GitHub repository in your code.	#
#Also we would like you to mention us and/or Team MAI when you talk about your code to others.											#
#Our email is: mai.studentrobotics@gmail.com																							#
#########################################################################################################################################

from sr.robot import *
from decimal import *
import time
from math import cos, sin, radians, pi, sqrt, atan

import threading

#ALL VALUES IN SAME UNITS: meter, degree, 

#Constants
ticksPerMeter = 3200#number of ticks per meter for the motors
wheelCircumference = 0.3141#circumference of a single wheel
robotCircumference = 1.0838#circumference of the robot



motorSpeedFBFastFL = -49
motorSpeedFBFastBL = -49
motorSpeedFBFastFR = 50
motorSpeedFBFastBR = 51

motorSpeedFBSlowFL = -24
motorSpeedFBSlowBL = -22
motorSpeedFBSlowFR = 25
motorSpeedFBSlowBR = 25

motorSpeedLRFastFL = 49
motorSpeedLRFastBL = -50
motorSpeedLRFastFR = 49
motorSpeedLRFastBR = -50

motorSpeedLRSlowFL = 24
motorSpeedLRSlowBL = -26
motorSpeedLRSlowFR = 24
motorSpeedLRSlowBR = -23

backmotorSpeedFBFastFL = -49
backmotorSpeedFBFastBL = -49
backmotorSpeedFBFastFR = 50
backmotorSpeedFBFastBR = 51

backmotorSpeedFBSlowFL = -24
backmotorSpeedFBSlowBL = -22
backmotorSpeedFBSlowFR = 25
backmotorSpeedFBSlowBR = 25

backmotorSpeedLRFastFL = 49
backmotorSpeedLRFastBL = -50
backmotorSpeedLRFastFR = 49
backmotorSpeedLRFastBR = -50

backmotorSpeedLRSlowFL = 24
backmotorSpeedLRSlowBL = -26
backmotorSpeedLRSlowFR = 24
backmotorSpeedLRSlowBR = -23

motorSpeedTurnFL = 25
motorSpeedTurnBL = 25
motorSpeedTurnFR = 25
motorSpeedTurnBR = 25


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
	#print 'setMotor'
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
    distCorrect = (dist/2.0) * sqrt(2)#distance the wheels actually have to drive
    distTick = (abs(distCorrect)/wheelCircumference)*ticksPerMeter#distance in ticks
    print 'distTick: ' + str(distTick)
    currentDistTick = 0
    if dist > 1:
        setMotor(fl = motorSpeedFBSlowFL, bl = motorSpeedFBSlowBL, fr = motorSpeedFBSlowFR, br = motorSpeedFBSlowBR)
        time.sleep(0.5)
        setMotor(fl = motorSpeedFBFastFL, bl = motorSpeedFBFastBL, fr = motorSpeedFBFastFR, br = motorSpeedFBFastBR)
    elif dist > 0:
        setMotor(fl = motorSpeedFBSlowFL, bl = motorSpeedFBSlowBL, fr = motorSpeedFBSlowFR, br = motorSpeedFBSlowBR)
    elif dist < -1:
        setMotor(fl = -backmotorSpeedFBSlowFL, bl = -backmotorSpeedFBSlowBL, fr = -backmotorSpeedFBSlowFR, br = -backmotorSpeedFBSlowBR)
        time.sleep(0.5)
        setMotor(fl = -backmotorSpeedFBFastFL, bl = -backmotorSpeedFBFastBL, fr = -backmotorSpeedFBFastFR, br = -backmotorSpeedFBFastBR)
    elif dist <= 0:
        setMotor(fl = -backmotorSpeedFBSlowFL, bl = -backmotorSpeedFBSlowBL, fr = -backmotorSpeedFBSlowFR, br = -backmotorSpeedFBSlowBR)
    else:
        raise TypeError()
    while currentDistTick < distTick:
        currentDistTick += (getTicks('FL') + getTicks('FR'))/2
        if (distTick - currentDistTick) <  3200:
            if dist > 0:
                setMotor(fl = motorSpeedFBSlowFL, bl = motorSpeedFBSlowBL, fr = motorSpeedFBSlowFR, br = motorSpeedFBSlowBR)
            else:
                setMotor(fl = -backmotorSpeedFBSlowFL, bl = -backmotorSpeedFBSlowBL, fr = -backmotorSpeedFBSlowFR, br = -backmotorSpeedFBSlowBR)
    setMotor(fl = 0, bl = 0, fr = 0, br = 0)

def llDriveLR(dist):#positive: left
    distCorrect = (dist/2.0) * sqrt(2)#distance the wheels actually have to drive
    distTick = (abs(distCorrect)/wheelCircumference)*ticksPerMeter#distance in ticks
    print 'distTick: ' + str(distTick)
    currentDistTick = 0
    if dist > 1:
        setMotor(fl = motorSpeedLRSlowFL, bl = motorSpeedLRSlowBL, fr = motorSpeedLRSlowFR, br = motorSpeedLRSlowBR)
        time.sleep(0.5)
        setMotor(fl = motorSpeedLRFastFL, bl = motorSpeedLRFastBL, fr = motorSpeedLRFastFR, br = motorSpeedLRFastBR)
    elif dist > 0:
		setMotor(fl = motorSpeedLRSlowFL, bl = motorSpeedLRSlowBL, fr = motorSpeedLRSlowFR, br = motorSpeedLRSlowBR)
    elif dist < -1:
		setMotor(fl = -backmotorSpeedLRSlowFL, bl = -backmotorSpeedLRSlowBL, fr = -backmotorSpeedLRSlowFR, br = -backmotorSpeedLRSlowBR)
		time.sleep(0.5)
		setMotor(fl = -backmotorSpeedLRFastFL, bl = -backmotorSpeedLRFastBL, fr = -backmotorSpeedLRFastFR, br = -backmotorSpeedLRFastBR)
    elif dist <= 0:
		setMotor(fl = -backmotorSpeedLRSlowFL, bl = -backmotorSpeedLRSlowBL, fr = -backmotorSpeedLRSlowFR, br = -backmotorSpeedLRSlowBR)
    else:
		raise TypeError()
    while currentDistTick < distTick:
		currentDistTick += (getTicks('FL') + getTicks('BL'))/2
		if (distTick - currentDistTick) <  3200:
			if dist > 0:
				setMotor(fl = motorSpeedLRSlowFL, bl = motorSpeedLRSlowBL, fr = motorSpeedLRSlowFR, br = motorSpeedLRSlowBR)
			else:
				setMotor(fl = -backmotorSpeedLRSlowFL, bl = -backmotorSpeedLRSlowBL, fr = -backmotorSpeedLRSlowFR, br = -backmotorSpeedLRSlowBR)
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

def llGrab(arm, state):#'F'/'R'/'L'/'B' #True: closed
   if arm == 'F':
        if state == True:
            R.servos[0][7] = -40
        elif state == False:
            R.servos[0][7] = 80
   if arm == 'L':
        if state == True:
            R.servos[0][5] = 10
        elif state == False:
            R.servos[0][5] = -70
   if arm == 'B':
        if state == True:
            R.servos[0][0] = -10
        elif state == False:
            R.servos[0][0] = 70
   if arm == 'R':
        if state == True:
            R.servos[0][1] = -10
        elif state == False:
             R.servos[0][1] = 70
def llArmState(state):#'U'/'M'/'D'
    if state == 'U':
        R.servos[0][4] = -15
    elif state == 'M':
        R.servos[0][4] = -60
    elif state == 'D':
        R.servos[0][4] = -85
    elif state == 'MU':
        R.servos[0][4] = -45
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
	if motor == 'FL':
		return R.ruggeduinos[0].motorStatusFL()
	elif motor == 'BL':
		return R.ruggeduinos[0].motorStatusBL()
	elif motor == 'FR':
		return R.ruggeduinos[0].motorStatusFR()
	elif motor == 'BR':
		return R.ruggeduinos[0].motorStatusBR()
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
	def getRemainingTime(self):
		return remainingTime

t = Time(0, "time", 0)

#Constants
turnDict = {
	'z0id32':0, 'z0id33':0, 'z0id34':5, 'z0id35':1, 'z0id36':4, 'z0id37':3, 'z0id38':0, 'z0id39':0, 'z0id40':5, 'z0id41':4, 'z0id42':1, 'z0id43':3, 'z0id44':0, 'z0id45':0, 'z0id46':5, 'z0id47':3, 'z0id48':1, 'z0id49':4,
	'z1id32':1, 'z1id33':3, 'z1id34':3, 'z1id35':5, 'z1id36':1, 'z1id37':4, 'z1id38':2, 'z1id39':2, 'z1id40':4, 'z1id41':5, 'z1id42':3, 'z1id43':1, 'z1id44':3, 'z1id45':1, 'z1id46':1, 'z1id47':5, 'z1id48':4, 'z1id49':3,
	'z2id32':2, 'z2id33':2, 'z2id34':4, 'z2id35':3, 'z2id36':5, 'z2id37':1, 'z2id38':1, 'z2id39':3, 'z2id40':3, 'z2id41':1, 'z2id42':5, 'z2id43':4, 'z2id44':1, 'z2id45':3, 'z2id46':3, 'z2id47':4, 'z2id48':5, 'z2id49':1,
	'z3id32':3, 'z3id33':1, 'z3id34':1, 'z3id35':4, 'z3id36':3, 'z3id37':5, 'z3id38':3, 'z3id39':1, 'z3id40':1, 'z3id41':3, 'z3id42':4, 'z3id43':5, 'z3id44':2, 'z3id45':2, 'z3id46':4, 'z3id47':1, 'z3id48':3, 'z3id49':5
	}

abortTime = 90
#End Constants
#Variables
hasTokenF = False#0/False = keiner, 1/True = einer da, 2/True = einer richtig gedreht da
hasTokenL = False#0/False = keiner, 1/True = einer da, 2/True = einer richtig gedreht da
hasTokenB = False#0/False = keiner, 1/True = einer da, 2/True = einer richtig gedreht da
hasTokenR = False#0/False = keiner, 1/True = einer da, 2/True = einer richtig gedreht da

tokenNumber = 2#Number of Token to collect
#End Variables

def rps(m):#@Parameter ArenaMarker @return (x,y,rot)
    print 'rps ' + str(m)
    distToWall = cos((pi/180) * m.orientation.rot_y) * m.dist
    distToLeftWall = ((m.info.code % 7) + 1) + sin((pi/180) * m.orientation.rot_y) * m.dist 
    relrot = m.orientation.rot_y
    
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

def returnToZone():
	print 'returnToZone'
	m = lookToMarker(0, None)
	rpsInfo = rps(m)
	print '    ' + str(rpsInfo)
	fromZeroDegToCorner = (90+(180/pi)*atan(rpsInfo[0]/rpsInfo[1]))#(90 + arctan(x/y))
	turn(-(rpsInfo[2]+fromZeroDegToCorner))
	driveFB(sqrt(rpsInfo[0]**2 + rpsInfo[1]**2) - 1)

def sortForDist(markerArr):
    print 'sortForDist'
    newArr = []
    while len(markerArr) > 0:
        lowest = markerArr[0]
        for m in markerArr:
            if m.dist < lowest.dist:
                lowest = m
        markerArr.remove(lowest)
        newArr.append(lowest)
    return newArr
	
def lookToMarker(type = 1, prefDist = None):#0: Arena, 1: Token(U+B+S) #preferred distance to Marker +-0.3m
	print 'lookToMarker ' + str(type) + '@' + str(prefDist) + 'm'
	counter = 0
	while counter < 24 and not getRemainingTime < abortTime:
		markers = scan()
		if len(markers) == 0:
			turn(15)
		else:
			markers = sortForDist(markers)
			for m in markers:
				if (type == 0 and m.info.marker_type == MARKER_ARENA) or (type == 1 and m.info.marker_type == MARKER_TOKEN_TOP or m.info.marker_type == MARKER_TOKEN_SIDE or m.info.marker_type == MARKER_TOKEN_BOTTOM):
					return m
				else:
					turn(15)

def takeToken(prefDist = None):#go to and grab Token
    global hasTokenF, hasTokenL, hasTokenB, hasTokenR
    print 'takeToken @' + str(prefDist) + 'm'
    m = lookToMarker(1, prefDist)
    case = getOrientation(m)
    hyp = m.dist
    alpha = m.orientation.rot_y
    ankath = cos(radians(alpha)) * hyp
    ggkath = sin(radians(alpha)) * hyp
    if case == 1 or case == 3:
        turn(90-alpha)
        if alpha > 0:
            driveLR(-ankath)
        else:
            driveLR(-ankath)
        driveFB(ggkath - 0.20)
    else:
        turn(-alpha)
        driveLR(-ggkath)
        driveFB(ankath - 0.20)
    if not hasTokenB:
        turn(165)
        driveFB(-1)
        grab('B', True)
        hasTokenB = True
    elif not hasTokenF:
        armState('M')
        grab('F', False)
        time.sleep(0.5)
        driveFB(1)
        grab('F', True)
        time.sleep(0.5)
        armState('MU')
        time.sleep(1)
        hasTokenF = True
    elif not hasTokenL:
        turn(-45)
        driveLR(1)
        grab('L', True)
        hasTokenL = True
    elif not hasTokenR:
        turn(45)
        driveLR(1)
        grab('R', True)
        hasTokenR = True

def scanGrabbedToken():
	print 'scanGrabbedToken'
	print 'camState(D)'
	markers = scan()
	print 'camState(U)'
	return sortForDist(markers)[0]
	
def getOrientation(m):#sunny side up
    print 'getOrientation ' + str(m)
    try:
        baseCase = turnDict['z{0}id{1}'.format(zone(), m.info.code)]
    except KeyError:
            print 'wrong marker!!'
            return 5
    orientat = 0
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
	return case

def orientate():
    print 'orientate'
    global hasTokenF
    armState('M')
    time.sleep(1)
    case = getOrientation(scanGrabbedToken())
    if case == 0:#A - R'/R3
		orientationR()
		orientationR()
		orientationR()
    elif case == 1:#C - U R
        print 'regrabU'
        orientationR()
    elif case == 2:#D - R
        orientationR()
    elif case == 3:#B - U' R
        print 'regrab in den parameter gehoert V'
        orientationR()
    elif case == 4:#E - R2
        orientationR()
        orientationR()
    #elif case == 5:#F - /
	hasTokenF = 2

def orientationR():#R #we are asuming that the robot is holding the token in question or standing right in front of it
    print 'orientationR'
    grab('F', False)
    time.sleep(1)
    armState('D')
    time.sleep(1)
    grab('F', True)
    time.sleep(1)
    armState('U')
    time.sleep(2)
    grab('F', False)
    time.sleep(0.5)
    turn(1.5)
    time.sleep(0.2)
    armState('M')
    time.sleep(1)
    grab('F', True)
    time.sleep(0.5)

def swapToken():
    print 'swapToken'
    global hasTokenF, hasTokenL, hasTokenB, hasTokenR
    armState('M')
    time.sleep(1)
    if hasTokenL == 1:
		grab('F', False)
		time.sleep(0.5)
		driveFB(-0.3)
		turn(-90)
		grab('L', False)
		time.sleep(0.2)
		driveLR(-0.3)
		turn(90)
		driveFB(0.3)
		grab('F', True)
		time.sleep(0.5)
		turn(-90)
		driveLR(0.3)
		grab('L', True)
		time.sleep(0.3)
		turn(90)
		hasTokenL = hasTokenF
		hasTokenF = 1
		return True
    elif hasTokenR == 1:
		grab('F', False)
		time.sleep(0.5)
		driveFB(-0.3)
		turn(90)
		grab('R', False)
		time.sleep(0.2)
		driveLR(0.3)
		turn(-90)
		driveFB(0.3)
		grab('F', True)
		time.sleep(0.5)
		turn(90)
		driveLR(-0.3)
		grab('R', True)
		time.sleep(0.3)
		turn(90)
		hasTokenR = hasTokenF
		hasTokenF = 1
		return True
    elif hasTokenB == 1:
		grab('F', False)
		time.sleep(0.5)
		driveFB(-0.3)
		turn(180)
		grab('B', False)
		time.sleep(0.2)
		driveFB(0.3)
		turn(180)
		driveFB(0.3)
		grab('F', True)
		time.sleep(0.5)
		turn(180)
		driveFB(-0.3)
		grab('B', True)
		time.sleep(0.3)
		turn(180)
		hasTokenB = hasTokenF
		hasTokenF = 1
		return True
    return False

def firstToken(choice = 'None'):#'MM'/'FM'/'RM'
	print 'firstToken'
	if choice == 'MM':
		turn(-45)
		driveFB(2.25)
		turn(90)
		driveFB(2.5)
		turn(-40)
	if choice == 'FM':
		turn(-45)
		driveLR(3)
	if choice == 'RM':
		turn(45)
		driveLR(-3)
	else:
		print 'None chosen'

#for debugging and testing	
def test():
	raise NotImplementedError()
	
def test2():
	markers = ()
	while markers == 0:
		markers = scan()
	for m in markers:
		print(m.orientation.rot_y)
		
def test3():
    global hasTokenB
    hasTokenB = True
    hasTokenR = True
    hasTokenL = True
    takeToken()
    print 'sucessfully tested takeToken()'
	
def test4():
    armState('M')
    grab('F', False)
    print('insert token now')
    time.sleep(5)
    grab('F', True)
    hasTokenF = True
    time.sleep(1)
    armState('MU')
    time.sleep(5)
    orientate()
    print 'sucessfully tested orientate()'
	
def test5():
    markers = scan()
    for m in markers:
        print(m.dist)
        print(m.orientation.rot_y)
        print('------------------------------------')
    driveFB(2)
    time.sleep(0)
    driveFB(-2)
    time.sleep(0)
    markers = scan()
    for m in markers:
        print(m.dist)
        print('-------------------------------------')
    driveLR(1)
    time.sleep(0)
    driveLR(-1)
    markers = scan()
    for m in markers:
		print(m.dist)
		print(m.orientation.rot_y)
		print('-------------------------------------')
        
def test6():
    crossStateInfo = None
    markers = ()
    while markers == 0:
        markers = scan()
    for m in markers:
        crossStateInfo = m
        turn(crossStateInfo)
        time.sleep(2)
        turn(-crossStateInfo)
        time.sleep(2)
        turn(90-crossStateInfo)     
#/for debugging and testing
	

def initialize():
	grab('L', False)
	grab('B', False)
	grab('R', False)
	print 'Hello, World!'
	
#main tactic
def main():
	firstToken('MM')
	takeToken()
	takeToken()
	returnToZone()
	while swapToken():
		orientate()
	grab('F', False)
	grab('L', False)
	grab('B', False)
	grab('R', False)
	print('Done')
#/main tactic
#initialize()
#main()

#test()
#test2()
#test3()
test4()
#test5()
#test6()