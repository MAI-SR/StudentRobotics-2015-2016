#########################################################################################################################################
#Disclaimer:                        																									#
#---------------------------------------------------------------------------------------------------------------------------------------#
#You can use this code for any and all educational purposes.																			#
#If you want to use it for the Student Robotics Contest you will have to send us an email and link our GitHub repository in your code.	#
#Also we would like you to mention us and/or Team MAI when you talk about your code to others.											#
#Our email is: mai.studentrobotics@gmail.com																							#
#########################################################################################################################################

from ll import *
from decimal import *
from time import sleep
from math import cos, sin, radians, pi, sqrt

import threading
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
	print 'takeToken@' + str(prefDist) + 'm'
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
			driveLR(ankath)
		driveFB(ggkath - 0.3)
	else:
		turn(-alpha)
		driveLR(ggkath)
		driveFB(ankath - 0.3)
	
	if not hasTokenB:
		turn(180)
		driveFB(-0.5)
		grab('B', True)
		hasTokenB = True
	else:
		grab('F', False)
		sleep(0.5)
		driveFB(0.5)
		grab('F', True)
		hasTokenF = True
	
def scanGrabbedToken():
	print 'scanGrabbedToken'
	camState('D')
	scan()
	camState('U')
	return sortForDist(markers)[0]
	
def getOrientation(m):#sunny side up
    print 'getOrientation ' + str(m)
	baseCase = turnDict['z{0}id{1}'.format(zone(), m.info.code)]
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
	case = getOrientation()
    if case == 0:#A - R'/R3
		orientationR()
		orientationR()
		orientationR()
    elif case == 1:#C - U R
        regrab('U')
        orientationR()
    elif case == 2:#D - R
        orientationR()
    elif case == 3:#B - U' R
        regrab('V')
        orientationR()
    elif case == 4:#E - R2
        orientationR()
		orientationR()
    #elif case == 5:#F - /
	hasTokenF = 2

def orientationR():#R #we are asuming that the robot is holding the token in question or standing right in front of it
	print 'orientationR'
	grabSide('front', False)
	sleep(0.5)
	moveArm('down')
	sleep(1)
	grabSide('front', True)
	sleep(1)
	moveArm('up')
	sleep(2)
	grabSide('front', False)
	sleep(0.5)
	turn(1.5)
	sleep(0.2)
	moveArm('middle')
	sleep(1)
	grabSide('front', True)
	sleep(0.5)

def swapToken():
	print 'swapToken'
	global hasTokenF, hasTokenL, hasTokenB, hasTokenR
	if hasTokenL == 1:
		grab('F', False)
		sleep(0.5)
		driveFB(-0.3)
		turn(-90)
		grab('L', False)
		sleep(0.2)
		driveLR(-0.3)
		turn(90)
		driveFB(0.3)
		grab('F', True)
		sleep(0.5)
		turn(-90)
		driveLR(0.3)
		grab('L', True)
		sleep(0.3)
		turn(90)
		hasTokenL = hasTokenF
		hasTokenF = 1
		return True
	elif hasTokenR == 1:
		grab('F', False)
		sleep(0.5)
		driveFB(-0.3)
		turn(90)
		grab('R', False)
		sleep(0.2)
		driveLR(0.3)
		turn(-90)
		driveFB(0.3)
		grab('F', True)
		sleep(0.5)
		turn(90)
		driveLR(-0.3)
		grab('R', True)
		sleep(0.3)
		turn(90)
		hasTokenR = hasTokenF
		hasTokenF = 1
		return True
	elif hasTokenB == 1:
		grab('F', False)
		sleep(0.5)
		driveFB(-0.3)
		turn(180)
		grab('B', False)
		sleep(0.2)
		driveFB(0.3)
		turn(180)
		driveFB(0.3)
		grab('F', True)
		sleep(0.5)
		turn(180)
		driveFB(-0.3)
		grab('B', True)
		sleep(0.3)
		turn(180)
		hasTokenB = hasTokenF
		hasTokenF = 1
		return True
	return False

def firstToken(choice = 'Nah'):#'MM'/'FM'/'RM'
	print 'firstToken'
	if choice == 'MM':
		turn(-45)
		driveFB(2.5)
		turn(90)
		driveFB(2.5)
		turn(-50)
	if choice == 'FM':
		turn(-45)
		driveLR(3)
	if choice == 'RM':
		turn(45)
		driveLR(-3)
	else:
		print 'None chosen'

	
def test():
	raise NotImplementedError()

def setup():
	grab('L', False)
	grab('B', False)
	grab('R', False)
	print 'Hello, World!"

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
#main
setup()
main()