#########################################################################################################################################
#Disclaimer:                                                    			#
#---------------------------------------------------------------------------------------------------------------------------------------#
#You can use this code for any and all educational purposes.																			#
#If you want to use it for the Student Robotics Contest you will have to send us an email and link our GitHub repository in your code.	#
#Also we would like you to mention us and/or Team MAI when you talk about your code to others.											#
#Our email is: mai.studentrobotics@gmail.com																							#
#########################################################################################################################################

from sr.robot import *
from decimal import *
import time
from math import cos, sin, radians, pi, sqrt, atan, asin, degrees

import threading

#ALL VALUES IN SAME UNITS: meter, degree

#Constants
ticksPerMeter = 3200#number of ticks per meter for the motors
wheelCircumference = 0.3141#circumference of a single wheel
robotCircumference = 1.021018#circumference of the robot
#robotCircumference2 = 0.958186#circumference of the robot

motorSpeedFBFastFL = -49
motorSpeedFBFastBL = -49
motorSpeedFBFastFR = 50
motorSpeedFBFastBR = 51

motorSpeedFBSlowFL = motorSpeedFBFastFL
motorSpeedFBSlowBL = motorSpeedFBFastBL
motorSpeedFBSlowFR = motorSpeedFBFastFR
motorSpeedFBSlowBR = motorSpeedFBFastBR

motorSpeedLRFastFL = 49
motorSpeedLRFastBL = -51
motorSpeedLRFastFR = 51
motorSpeedLRFastBR = -51

motorSpeedLRSlowFL = motorSpeedLRFastFL
motorSpeedLRSlowBL = motorSpeedLRFastBL
motorSpeedLRSlowFR = motorSpeedLRFastFR
motorSpeedLRSlowBR = motorSpeedLRFastBR

backmotorSpeedFBFastFL = -49
backmotorSpeedFBFastBL = -49
backmotorSpeedFBFastFR = 50
backmotorSpeedFBFastBR = 51

backmotorSpeedFBSlowFL = backmotorSpeedFBFastFL
backmotorSpeedFBSlowBL = backmotorSpeedFBFastBL
backmotorSpeedFBSlowFR = backmotorSpeedFBFastFR
backmotorSpeedFBSlowBR = backmotorSpeedFBFastBR

backmotorSpeedLRFastFL = 49
backmotorSpeedLRFastBL = -51
backmotorSpeedLRFastFR = 50
backmotorSpeedLRFastBR = -51

backmotorSpeedLRSlowFL = backmotorSpeedLRFastFL
backmotorSpeedLRSlowBL = backmotorSpeedLRFastBL
backmotorSpeedLRSlowFR = backmotorSpeedLRFastFR
backmotorSpeedLRSlowBR = backmotorSpeedLRFastBR

motorSpeedTurnFast = 25
motorSpeedTurnSlow = 25#15

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
    time.sleep(0.25)

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
    time.sleep(0.25)

#End Drive
#Turn
def llTurn(degree):#positive: counter clockwise
    while degree > 180:
        degree -= 360
    while degree < -180:
        degree += 360
    currentDegreeTick = 0.0
    degreeTick = 0.0
    degreeTick = float(((abs(degree/360.0)*robotCircumference)/wheelCircumference)*ticksPerMeter)
    resetTicks()
    if degree > 0:
        setMotor(fl = motorSpeedTurnFast, bl = motorSpeedTurnFast, fr = motorSpeedTurnFast, br = motorSpeedTurnFast)
    else:
        setMotor(fl = -motorSpeedTurnFast, bl = -motorSpeedTurnFast, fr = -motorSpeedTurnFast, br = -motorSpeedTurnFast)
    while currentDegreeTick < degreeTick-500:
        currentDegreeTick += (getTicks('FR')+getTicks('BL'))/2.0
    if degree > 0:
        setMotor(fl = motorSpeedTurnSlow, bl = motorSpeedTurnSlow, fr = motorSpeedTurnSlow, br = motorSpeedTurnSlow)
    else:
        setMotor(fl = -motorSpeedTurnSlow, bl = -motorSpeedTurnSlow, fr = -motorSpeedTurnSlow, br = -motorSpeedTurnSlow)
    while currentDegreeTick < degreeTick:
        currentDegreeTick += (getTicks('FR')+getTicks('BL'))/2.0
    setMotor(fl = 0, bl = 0, fr = 0, br = 0)
    time.sleep(0.25)
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
        print 'this arm was taken off the robot due to resons... please rewrite your code to stop using this arm alltogether'
   if arm == 'R':
        print 'this arm was taken off the robot due to resons... please rewrite your code to stop using this arm alltogether'
def llArmState(state):#'U'/'M'/'MU'/'D'
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
def armState(state):#'U'/'M'/'MU'/'D'
	print 'armState ' + str(state)
	return llArmState(state)
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
        global remainingTime
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
prefDist = None
case = None

hasTokenF = False

tmp = {None, None, None, None}

alpha = 0
hyp = 0
ankath = 0
ggkath = 0

prefDist = None

def rps(m):#@Parameter ArenaMarker @return (x,y,rot)
    print 'rps ' + str(m)
    distToWall = cos(radians(m.orientation.rot_y)) * m.dist
    distToLeftWall = ((m.info.code % 7) + 1) + sin(radians(m.orientation.rot_y)) * m.dist 
    alpha = m.orientation.rot_y
    if m.info.code >= 0 and m.info.code <= 6:
            if R.zone == 0:
                return (distToWall, distToLeftWall, 180 - alpha, -alpha)
            elif R.zone == 1:
                return (8-distToLeftWall, distToWall, 90 - alpha, -alpha)
            elif R.zone == 2:
                return (8-distToWall, 8-distToLeftWall, 0 - alpha, -alpha)
            elif R.zone == 3:
                return (distToLeftWall, 8-distToWall, -90 - alpha, -alpha)
    elif m.info.code >= 7 and m.info.code <= 13:
            if R.zone == 0:
                return (distToLeftWall, 8-distToWall, -90 - alpha, -alpha)
            elif R.zone == 1:
                return (distToWall, distToLeftWall, 180 - alpha, -alpha)
            elif R.zone == 2:
                return (8-distToLeftWall, distToWall, 90 - alpha, -alpha)
            elif R.zone == 3:
                return (8-distToWall, 8-distToLeftWall, 0 - alpha, -alpha)
    elif m.info.code >= 14 and m.info.code <= 20:
            if R.zone == 0:
                return (8-distToWall, 8-distToLeftWall, 0 - alpha, -alpha)
            elif R.zone == 1:
                return (distToLeftWall, 8-distToWall, -90 - alpha, -alpha)
            elif R.zone == 2:
                return (distToWall, distToLeftWall, 180 - alpha, -alpha)
            elif R.zone == 3:
                return (8-distToLeftWall, distToWall, 90 - alpha, -alpha)
    elif m.info.code >= 21 and m.info.code <= 27:
            if R.zone == 0:
                return (8-distToLeftWall, distToWall, 90 - alpha, -alpha)
            elif R.zone == 1:
                return (8-distToWall, 8-distToLeftWall, 0 - alpha, -alpha)
            elif R.zone == 2:
                return (distToLeftWall, 8-distToWall, -90 - alpha, -alpha)
            elif R.zone == 3:
                return (distToWall, distToLeftWall, 180 - alpha, -alpha)

def returnToZone():
    print 'returnToZone'
    m = lookToMarker(0, None, 1)
    rpsInfo = rps(m)
    hyp = sqrt(rpsInfo[0]**2 + rpsInfo[1]**2)
    turn(rpsInfo[2] + -90 + -degrees(atan(rpsInfo[0]/rpsInfo[1])))
    if hyp >= 1.5:
        driveFB((hyp / 2)*(-1))
        m = lookToMarker(0, None, 1)
        rpsInfo = rps(m)
        hyp = sqrt(rpsInfo[0]**2 + rpsInfo[1]**2)
        turn(rpsInfo[2] + -90 + -degrees(atan(rpsInfo[0]/rpsInfo[1])))
        driveFB((hyp - 1)*(-1))
    else:
        driveFB((hyp - 1)*(-1))

def sortForDist(markerArr):
    print 'sortForDist'
    lastlowest = None
    lastlastlowest = None
    newArr = []
    while len(markerArr) > 0:
        lowest = markerArr[0]
        for m in markerArr:
            if m.dist < lowest.dist:
                lastlastlowest = lastlowest
                lastlowest = lowest
                lowest = m
        if lowest is not None and getOrientation(lowest) == 0:
            newArr.append(lowest)
            markerArr.remove(lowest)
            print newArr
            return newArr
        elif lastlowest is not None and getOrientation(lastlowest) == 0:
            newArr.append(lastlowest)
            markerArr.remove(lastlowest)
            print newArr
            return newArr
        elif lastlastlowest is not None and getOrientation(lastlastlowest) == 0:
            newArr.append(lastlastlowest)
            markerArr.remove(lastlastlowest)
            print newArr
            return newArr
        else:
            newArr.append(lowest)
            markerArr.remove(lowest)
            print newArr
            return newArr

def lookToMarker(type = 1, prefDist = None, direction = 1):#0: Arena, 1: Token(U+B+S) #preferred distance to Marker +-0.3m #direction = turndirection; pls only use 1 or -1
    print 'lookToMarker ' + str(type) + '@' + str(prefDist) + 'm'
    counter = 0
    marker = None
    while counter < 24:
        counter += 1
        markers = scan()
        if len(markers) == 0:
            turn(direction * 15)
        else:
            markers = sortForDist(markers)
            print 'sortiert'
            for m in markers:
                if (type == 0 and m.info.marker_type == MARKER_ARENA) or (type == 1 and (m.info.marker_type == MARKER_TOKEN_TOP or m.info.marker_type == MARKER_TOKEN_SIDE or m.info.marker_type == MARKER_TOKEN_BOTTOM)):
                    marker = m
                    print 'break'
                    break
                else:
                    turn(direction * 15)
        if marker != None:
            break
    if marker == None:
        driveFB(1)
        turn(-90)
        return lookToMarker(type, prefDist, direction)
    return marker
    
def searchAdjustMarker(direction):
    global prefDist
    counter = 0
    while counter <= 12:
        counter += 1
        markers = scan()
        for m in markers:
            markers = sortForDist(markers)
            print 'sortiert'
            for m in markers:
                if (type == 0 and m.info.marker_type == MARKER_ARENA) or (type == 1 and (m.info.marker_type == MARKER_TOKEN_TOP or m.info.marker_type == MARKER_TOKEN_SIDE or m.info.marker_type == MARKER_TOKEN_BOTTOM)):
                    return m
        if counter == 6:
            driveFB(0.30)
        if counter >= 12:
            m = lookToMarker(1, prefDist, direction)
            counter = 0
            return m

def takeToken(direction):
    global alpha, hyp, ankath, ggkath, prefDist, tmp
    alpha = 0
    hyp = 0
    ankath = 0
    ggkath = 0
    m = lookToMarker(1, prefDist, direction)
    case = getOrientation(m)
    alpha = m.orientation.rot_y
    hyp = m.dist
    ankath = cos(radians(alpha)) * hyp
    ggkath = sin(radians(alpha)) * hyp
    if case != 1 or case != 3:
        turn(-alpha)
        driveLR(-ggkath)
    else:
        if hyp >= 0.57:
            if alpha > 0:
                turn(90 - alpha)
                driveLR(-ankath - 0.125)
            else:
                turn(-90 - alpha)
                driveLR(ankath + 0.125)
        else:
            if alpha > 0:
                turn(90 - alpha)
                driveFB(0.25)
                driveLR(-ankath - 0.125)
            else:
                turn(-90 - alpha)
                driveFB(0.25)
                driveLR(ankath + 0.125)
    print 'finished takeToken(Step 1)'
    tmp = adjust(direction)
    print 'finished takeToken(Step 2)'
    if tmp[1] <= 0.5:
        driveFB(-tmp[1] + 0.58)
        turn(175)
        time.sleep(0.5)
        driveFB(0.58 + 0.15)
        grab('F', True)
        grab('F', False)
        driveFB(0.3)
        grab('F', True)
        time.sleep(0.25)
        armState('MU')
        hasTokenF = True
        hasFirstToken = True
    elif tmp[1] <= 1:
        driveFB(-tmp[1] / 2 + 0.30)
        tmp = adjust(direction)
        alpha = m.orientation.rot_y
        hyp = m.dist
        ankath = cos(radians(alpha)) * hyp
        ggkath = sin(radians(alpha)) * hyp
        driveFB(-tmp[1] + 0.58)
        turn(175)
        time.sleep(0.5)
        driveFB(0.58 + 0.30)
        grab('F', True)
        grab('F', False)
        driveFB(0.3)
        grab('F', True)
        time.sleep(0.25)
        armState('MU')
        hasTokenF = True
        hasFirstToken = True
    elif tmp[1] <= 2:
        driveFB(-tmp[1] / 3)
        tmp = adjust(direction)
        alpha = m.orientation.rot_y
        hyp = m.dist
        ankath = cos(radians(alpha)) * hyp
        ggkath = sin(radians(alpha)) * hyp
        driveFB(-tmp[1] / 2 + 0.30)
        tmp = adjust(direction)
        alpha = m.orientation.rot_y
        hyp = m.dist
        ankath = cos(radians(alpha)) * hyp
        ggkath = sin(radians(alpha)) * hyp
        driveFB(-tmp[1] + 0.58)
        turn(175)
        time.sleep(0.5)
        driveFB(0.58 + 0.30)
        grab('F', True)
        grab('F', False)
        driveFB(0.3)
        grab('F', True)
        time.sleep(0.25)
        armState('MU')
        hasTokenF = True
        hasFirstToken = True

def adjust(direction):
    global alpha, hyp, ankath, ggkath, prefDist, case
    m = searchAdjustMarker(direction)
    alpha = m.orientation.rot_y
    hyp = m.dist
    ankath = cos(radians(alpha)) * hyp
    ggkath = sin(radians(alpha)) * hyp
    while m.orientation.rot_y <= -0.15 or m.orientation.rot_y >= 0.15:
        turn(-alpha)
        driveLR(-ggkath)
        m = searchAdjustMarker(direction)
        alpha = m.orientation.rot_y                                         
        hyp = m.dist
        ankath = cos(radians(alpha)) * hyp
        ggkath = sin(radians(alpha)) * hyp
        if m.orientation.rot_y >= -0.15 or m.orientation.rot_y <= 0.15:
            case = getOrientation(m)
            return alpha, ankath, ggkath, case
    if m.orientation.rot_y >= -0.15 or m.orientation.rot_y <= 0.15:
        case = getOrientation(m)
        return alpha, ankath, ggkath, case
    
def getOrientation(m):#sunny side up
    print m.orientation.rot_z
    try:
        baseCase = turnDict['z{0}id{1}'.format(zone(), m.info.code)]
        print baseCase
    except KeyError:
            print 'wrong marker!!'
            return 0
    orientat = 0
    case = 0
    
    if m.orientation.rot_z < 45 and m.orientation.rot_z > -45:#Orientation F
        orientat += 0
        print'orientation = 0'
    elif m.orientation.rot_z < -45 and m.orientation.rot_z > -135:#Orientation R
    	orientat += 1
        print'orientation = 1'
    elif m.orientation.rot_z > -135 and m.orientation.rot_z < -225:#Orientation L
    	orientat += 2
        print'orientation = 2'
    elif m.orientation.rot_z < 100 and m.orientation.rot_z > 45:#Orientation B
    	orientat += 3
        print'orientation = 3'
    
    if baseCase == 0 or baseCase == 1 or baseCase == 2 or baseCase == 3:
    	case = (baseCase + orientat) % 4
    elif baseCase == 4 or baseCase == 5:
        case = baseCase
    print case
    return case
 
def orientate(case):
    global hasTokenF, tmp
    print 'orientate'
    armState('M')
    time.sleep(1)
    case = tmp[3]
    if case == 4:#A - R'/R3
		orientationR()
		orientationR()
		orientationR()
    elif case == 1:#C - U R
        print 'regrabU'
        orientationR()
    elif case == 5:#D - R
        orientationR()
    elif case == 3:#B - U' R
        print 'regrab in den parameter gehoert V'
        orientationR()
    elif case == 2:#E - R2
        orientationR()
        orientationR()
    #elif case == 0:#F - /
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
    time.sleep(0.7)
    armState('M')
    time.sleep(1)
    grab('F', True)
    time.sleep(0.5)

def initialize():
    armState('M')
    time.sleep(0.25)
    grab('F', False)
    print 'Hello, World!'

def main(tactic):
    global tmp
    if tactic == 'right':
        initialize()
        driveFB(-3.5)
        turn(80)
        driveFB(-0.5)
        while True:
            takeToken(1)
            driveFB(-0.40)
            orientate(tmp[3])
            initialize()
            driveFB(-0.5)
    elif tactic == 'left':
        initialize()
        driveFB(-3)
        turn(-80)
        driveFB(-0.5)
        while True:
            takeToken(1)
            driveFB(-0.40)
            orientate(tmp[3])
            initialize()
            driveFB(-0.5)
    elif tactic == 'dontusemeunlessyouhavefullpermission':
        initialize()
        driveLR(-3.5)
        driveFB(-0.50)
        takeToken(1)
        orientate(tmp[3])
        returnToZone()
        turn(-180)
        initialize()
        driveFB(0.2)
        driveFB(-3)
        takeToken(1)
        driveFB(-0.40)
        orientate(tmp[3])
        returnToZone()
        turn(-180)
        initialize()
    else:
        initialize()
        driveFB(-3)
        turn(80)
        driveFB(-0.5)
        while True:
            takeToken(1)
            driveFB(-0.40)
            orientate(tmp[3])
            initialize()
            driveFB(-0.5)


def test():
    while True:
        markers = scan()
        for m in markers:
            getOrientation(m)
    
def drivetest():
    driveFB(2)
    driveFB(-2)
    driveLR(2)
    driveLR(-2)
    turn(180)
    turn(-180)
    turn(90)
    turn(-90)

main('dontusemeunlessyouhavefullpermission')