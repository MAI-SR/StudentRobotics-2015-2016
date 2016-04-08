#########################################################################################################################################
#Disclaimer:                                    																						#
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
robotCircumference = 1.021018#circumference of the robot
#robotCircumference2 = 0.958186#circumference of the robot

alpha = 0
hyp = 0
ankth = 0
ggkath = 0
case = 0

tries = 0

hasAnotherToken = False

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

motorSpeedTurnFast = 25
motorSpeedTurnSlow = 15

lastTime = 0


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
    resetTicks()
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
    resetTicks()
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
    global lastTime
    while degree > 180:
        degree -= 360
    while degree < -180:
        degree += 360
    currentDegreeTick = 0.0
    degreeTick = 0.0
    degreeTick = float(((abs(degree/360.0)*robotCircumference)/wheelCircumference)*ticksPerMeter)
    lastTicks = 0
    lastTime = 0
    resetTicks()
    if degree > 0:
        setMotor(fl = motorSpeedTurnFast, bl = motorSpeedTurnFast, fr = motorSpeedTurnFast, br = motorSpeedTurnFast)
    else:
        setMotor(fl = -motorSpeedTurnFast, bl = -motorSpeedTurnFast, fr = -motorSpeedTurnFast, br = -motorSpeedTurnFast)
    while currentDegreeTick < degreeTick-500:
        currentDegreeTick += (getTicks('FR')+getTicks('BL'))/2.0
    while currentDegreeTick < degreeTick:#if under 500 to go
        currentDegreeTick += (getTicks('FR')+getTicks('BL'))/2.0
        if onceASec():#once a sec and the first time around
            if lastTicks == 0:#if the first time go slow
                if degree > 0:
                    setMotor(fl = motorSpeedTurnSlow, bl = motorSpeedTurnSlow, fr = motorSpeedTurnSlow, br = motorSpeedTurnSlow)
                else:
                    setMotor(fl = -motorSpeedTurnSlow, bl = -motorSpeedTurnSlow, fr = -motorSpeedTurnSlow, br = -motorSpeedTurnSlow)
                lastTicks = currentDegreeTick
                continue
            if lastTicks+250 > currentDegreeTick:#if we havent driven at least 250
                if degree > 0:
                    setMotor(fl = motorSpeedTurnFast, bl = motorSpeedTurnFast, fr = motorSpeedTurnFast, br = motorSpeedTurnFast)
                else:
                    setMotor(fl = -motorSpeedTurnFast, bl = -motorSpeedTurnFast, fr = -motorSpeedTurnFast, br = -motorSpeedTurnFast)
                lastTicks = currentDegreeTick
            else:#if we have driven at least 250 since 1 sec ago go slow
                if degree > 0:
                    setMotor(fl = motorSpeedTurnSlow, bl = motorSpeedTurnSlow, fr = motorSpeedTurnSlow, br = motorSpeedTurnSlow)
                else:
                    setMotor(fl = -motorSpeedTurnSlow, bl = -motorSpeedTurnSlow, fr = -motorSpeedTurnSlow, br = -motorSpeedTurnSlow)
                lastTicks = currentDegreeTick
    setMotor(fl = 0, bl = 0, fr = 0, br = 0)

def onceASec():#if you use this make sure to always put lastTime = 0 into your methode befor using this in order to make sure that this also triggers the first time its called
    global lastTime
    if lastTime == 0:
        lastTime = getRemainingTime()
        return True
    if lastTime-1 >= getRemainingTime():
        lastTime = getRemainingTime()
        return True
    else:
        return False

#End Turn
#End Motors

def llGrab(arm, state):#'F'/'R'/'L'/'B' #True: closed
   if arm == 'F':
        if state == True:
            R.servos[0][7] = -100
            R.servos[0][5] = 100
        elif state == False:
            R.servos[0][7] = 100
            R.servos[0][5] = -100
   if arm == 'L':
        print 'this arm was taken off the robot due to resons... please rewrite your code to stop using this arm alltogether'
   if arm == 'B':
        if state == True:
            R.servos[0][0] = -10
        elif state == False:
            R.servos[0][0] = 70
   if arm == 'R':
        print 'this arm was taken off the robot due to resons... please rewrite your code to stop using this arm alltogether'
def llArmState(state):#'U'/'M'/'D'
    print state
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
		R.servos[0][3] = 0
	elif state == 'D':
		R.servos[0][3] = -60
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

def resetTicks():
    R.ruggeduinos[0].motorStatusFL()
    R.ruggeduinos[0].motorStatusBL()
    R.ruggeduinos[0].motorStatusFR()
    R.ruggeduinos[0].motorStatusBR()
    

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
hasTokenB = True#0/False = keiner, 1/True = einer da, 2/True = einer richtig gedreht da

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
    m = lookToMarker(0, None, 1)
    rpsInfo = rps(m)
    hyp = sqrt(rpsInfo[0]**2 + rpsInfo[1]**2)
    print '    ' + str(rpsInfo)
    fromZeroDegToCorner = (90+(180/pi)*atan(rpsInfo[0]/rpsInfo[1]))#(90 + arctan(x/y))
    turn(-(rpsInfo[2]+fromZeroDegToCorner))
    if hyp >= 3:
        driveFB(hyp - hyp/2)
        m = lookToMarker(0, None, 1)
        rpsInfo = rps(m)
        hyp = sqrt(rpsInfo[0]**2 + rpsInfo[1]**2)
        print '    ' + str(rpsInfo)
        fromZeroDegToCorner = (90+(180/pi)*atan(rpsInfo[0]/rpsInfo[1]))#(90 + arctan(x/y))
        turn(-(rpsInfo[2]+fromZeroDegToCorner))
        driveFB(hyp - 1)
    else:
        driveFB(hyp - 1)

def sortForDist(markerArr):
    print 'sortForDist'
    newArr = []
    while len(markerArr) > 0:
        lowest = markerArr[0]
        for m in markerArr:
            if m.dist < lowest.dist:
                lowest = m
        newArr.append(lowest)
        markerArr.remove(lowest)
    return newArr
	
def lookToMarker(type = 1, prefDist = None, direction = 1):#0: Arena, 1: Token(U+B+S) #preferred distance to Marker +-0.3m #direction = turndirection; pls only use 1 or -1
    print 'lookToMarker ' + str(type) + '@' + str(prefDist) + 'm'
    counter = 0
    while counter < 24 and not getRemainingTime < abortTime:
        counter += 1
        markers = scan()
        if len(markers) == 0:
            turn(direction * 15)
        else:
            markers = sortForDist(markers)
            for m in markers:
                if (type == 0 and m.info.marker_type == MARKER_ARENA) or (type == 1 and (m.info.marker_type == MARKER_TOKEN_TOP or m.info.marker_type == MARKER_TOKEN_SIDE or m.info.marker_type == MARKER_TOKEN_BOTTOM)):
                    return m
                else:
                    turn(direction * 15)
                    
def resetConstants(): #just for safety
    global hyp, alpha, ankth, ggkath
    hyp = 0
    alpha = 0
    ankath = 0
    ggkath = 0
					
def takeToken(prefDist = None):
    global hasTokenF, hasTokenB, hyp, alpha, ankth, ggkath, tries, hasAnotherToken, case
    m = lookToMarker(1, prefDist, 1) #search for a token
    case = getOrientation(m)
    hyp = m.dist
    alpha = m.orientation.rot_y
    ankath = cos(radians(alpha)) * hyp
    ggkath = sin(radians(alpha)) * hyp
    if correctDriveB():
        m = lookToMarker(1, prefDist, 1) #search for a token
        case = getOrientation(m)
        hyp = m.dist
        alpha = m.orientation.rot_y
        ankath = cos(radians(alpha)) * hyp
        ggkath = sin(radians(alpha)) * hyp
    if case == 1 or case == 3: #right side is on non rotable side
        if alpha < 0: #position is on the left side of the marker
            turn(-90 - alpha)
            driveLR(ankath)
        else: #position is on the right side of the marker
            turn(90 - alpha)
            driveLR(-ankath)
    else: #right side is on rotable side
        turn(-alpha)
        if alpha < 0:
			driveLR(ggkath)
        else:
            driveLR(ggkath)
    print 'finished takeTokenStep1()'
    tmp = correctPosition(prefDist) #save the varibles of the position right in front of the marker in tmp
    if hasAnotherToken == True:
        hasAnotherToken = False
        return
    print 'finished takeTokenStep2()'
    if not case == 1 or not case == 3:
        if alpha < 0:
            print 'alpha < 0'    
        elif alpha > 0:
            print 'alpha > 0'
        if tmp[1] >= 3: #drive only half of the way in order to correct the position again
            driveFB(tmp[1]/2)
            tmp = correctPosition()
            if hasAnotherToken == True:
                hasAnotherToken = False
                return
        driveFB(tmp[1] - 0.20)
    if alpha < 0:
        print 'alpha < 0'
        if -tmp[2] >= 3: #drive only half of the way in order to correct the position again
            driveFB(-tmp[2] - tmp[2]/2)
            tmp = correctPosition()
            if hasAnotherToken == True:
                hasAnotherToken = False
                return
        driveFB(-tmp[2] - 0.20)
    elif alpha > 0:
        print 'alpha > 0'
        if tmp[2] >= 3: #drive only half of the way in order to correct the position again
            driveFB(tmp[2] - tmp[2]/2)
            tmp = correctPosition()
            if hasAnotherToken == True:
                hasAnotherToken = False
                return
        driveFB(tmp[2] - 0.20)
    if not hasTokenB:
        turn(180)
        driveFB(-1)
        grab('B', True)
        hasTokenB = True
    elif not hasTokenF:
        armState('M')
        grab('F', False)
        time.sleep(0.5)
        driveFB(0.5)
        grab('F', True)
        time.sleep(0.5)
        armState('MU')
        time.sleep(1)
        hasTokenF = True
        
def correctPosition(prefDist = None):
    global hyp, alpha, ankath, ggkath, tries
    m = lookToMarker(1, prefDist, 1)
    hyp = m.dist
    alpha = m.orientation.rot_y
    ankath = cos(radians(alpha)) * hyp
    ggkath = sin(radians(alpha)) * hyp
    while m.orientation.rot_y < -1 or m.orientation.rot_y > 1:
        print 'need to correct the position'
        turn(-alpha)
        if alpha < 0:
            driveLR(ggkath)
        else:
            driveLR(-ggkath)
        tries += 1
        print str(tries) + ' try bzw. tries'
        m = lookToMarker(1, prefDist, 1)
        hyp = m.dist
        alpha = m.orientation.rot_y
        ankath = cos(radians(alpha)) * hyp
        ggkath = sin(radians(alpha)) * hyp
        if m.orientation.rot_y > -1 or m.orientation.rot_y < 1:
            print 'corrected position'
            print alpha
            print ankath
            print ggkath
            return alpha, ankath, ggkath
        if tries >= 2:
            print 'take different Token'
            getOtherToken()
            return
    if m.orientation.rot_y > -1 or m.orientation.rot_y < 1:
        print 'standing right'
        return alpha, ankath, ggkath 
        
def correctDriveB():
    global alpha, ankth, ggkath, case
    if case == 1 or case == 3:
        if alpha > 0:
            if ankath <= 0.50:
                turn(-alpha)
                driveFB(-0.50 + ankath)
                return True
            return False
        else:
            if ggkath <= 0.50:
                turn(-alpha)
                driveFB(-0.50 + ggkath)
                return True
            return False
    else:
        if ggkath <= 0.50:
            turn(-alpha)
            driveFB(-0.50 + ggkath)
            return True
        return False
        
def getOtherToken(prefDist = None):
    global tries, hasAnotherToken
    tries = 0
    if alpha > 0:
        turn(-90)
        m = lookToMarker(1, prefDist, -1)
    else:
        turn(90)
        m = lookToMarker(1, prefDist, 1)
    takeToken()
    hasAnotherToken = True

def scanGrabbedToken():
    print 'scanGrabbedToken'
    camState('D')
    time.sleep(0.5)
    markers = scan()
    time.sleep(0.5)
    camState('U')
    return sortForDist(markers)[0]
	
def getOrientation(m):#sunny side up
    print 'getOrientation ' + str(m.orientation.rot_y)
    try:
        baseCase = turnDict['z{0}id{1}'.format(zone(), m.info.code)]
        print baseCase
    except KeyError:
            print 'wrong marker!!'
            return 5
    orientat = 0
    case = 0
    
    if m.orientation.rot_y < 45 or m.orientation.rot_y > 315:#Orientation F
    	orientat += 0
        print'orientation = 0'
    elif m.orientation.rot_y < 315 or m.orientation.rot_y > 225:#Orientation R
    	orientat += 1
        print'orientation = 1'
    elif m.orientation.rot_y < 135 or m.orientation.rot_y > 45:#Orientation L
    	orientat += 2
        print'orientation = 2'
    elif m.orientation.rot_y < 225 or m.orientation.rot_y > 135:#Orientation B
    	orientat += 3
        print'orientation = 3'
    
    if baseCase == 0 or baseCase == 1 or baseCase == 2 or baseCase == 3:
    	case = (baseCase + orientat) % 4
    elif baseCase == 4 or baseCase == 5:
        case = baseCase
    print case
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
    if hasTokenB == 1:
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
		driveLR(-2.5)
		driveFB(2.75)
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
		print m.orientation.rot_y
		
def test3():
    global hasTokenB
    hasTokenB = True
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
    driveFB(1)
    time.sleep(0)
    driveFB(-1)
    time.sleep(0)
    markers = scan()
    for m in markers:
        print(m.dist)
        print('-------------------------------------')
    driveLR(0.5)
    time.sleep(0)
    driveLR(-0.5)
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
        
def test7():
    while True:
        markers = scan()
        if len(markers) > 0:
            for m in markers:
                print m.dist
                print m.orientation.rot_y
        else:
            print ':('
            
def test8():
    while True:
        turn(-180)
        time.sleep(2)
        turn(180)
        time.sleep(2)
        turn(180)
        time.sleep(2)
        turn(-180)
        time.sleep(2)
        
def test9():
    print 'test new arm'
    armState('M')
    time.sleep(3)
    grab('F', True)
    time.sleep(2)
    grab('F', False)
#/for debugging and testing
	

def initialize():
    grab('B', False)
    armState('MU')
    time.sleep(0.25)
    grab('F', False)
    print 'Hello, World!'
	
#main tactic
def main():
    firstToken('MM')
    camState('D')
    takeToken()
    #takeToken()
    camState('U')
    returnToZone()
    #while swapToken(): #this used to include orientate and is used to wohn better 
    orientate()
    grab('F', False)
    grab('B', False)
    print('Done')
#/main tactic
initialize()
main()

#test()
#test2()
#test3()
#test4()
#test5()
#test6()
#test7()
#test8()
#test9()


