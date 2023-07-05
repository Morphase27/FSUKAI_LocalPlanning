from re import T
import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import random
import math
import time
import LocalPlanning3_0

def ReadFile(name):
    FileData = []
    file = open(name +'.csv', 'r')
    for line in file:
        if "blue" in line or "yellow" in line or "orange" in line:
            if line != "" and line != " " and line != '\n':
                FileData.append(line)
    file.close()
    return FileData

def ClearFile(name2):
    file = open(name2 +'.csv', 'w')
    file.truncate()
    file.close()


def Separator(FileData):
    conesPoses = []
    conesColors = []
    for line in FileData:
        coma1 = line.find(',')
        Color = line[0: coma1]
        coma2 = line[coma1+1:].find(',')
        coneX = float(line[coma1 + 1: coma1 + coma2 + 1])
        coma3 = line.find('\n')
        coneY = float(line[coma1 + coma2 + 2: coma3])
        conePose = []
        conePose.append(coneX)
        conePose.append(coneY)
        conesColors.append(Color)
        conesPoses.append(conePose)
    conesPoses = np.array(conesPoses)
    return [conesPoses, conesColors]

def DistFinder(A, B):
    if A[0] >= B[0]:
        DistX = A[0] - B[0]
    else:
        DistX = B[0] - A[0]

    if A[1] >= B[1]:
        DistY = A[1] - B[1]
    else:
        DistY = B[1] - A[1]

    Dist= (DistX**2 + DistY**2)**0.5
    return Dist

def AngleChange(CarXY, Point, CarAngle):

    PointShifted = [Point[0]-CarXY[0], Point[1]-CarXY[1]]
    CarToPoint = DistFinder([0, 0], PointShifted)
    cosPoint = PointShifted[0] / 1.0/ CarToPoint
    PointAngle = np.arccos(cosPoint)
    PointAngle = PointAngle * 180/ math.pi
    if PointShifted[1] < 0:
        PointAngle = PointAngle * -1    

    AngleToCar = 90 - PointAngle - CarAngle 
    if AngleToCar > 180:
        AngleToCar = AngleToCar - 360
    elif AngleToCar < -180:
        AngleToCar = AngleToCar + 360
    
    return AngleToCar

def CarPosition(speed, t, path):
    travelled = t * speed
    totDist = 0
    i = 0
    while totDist < travelled and i < len(path) - 1:
        TotDist0 = totDist
        totDist = TotDist0 + DistFinder(path[i], path[i+1])
        i = i+1

    prevPoint = path[i - 1]
    nextPoint = path[i]

    if prevPoint[0] >= nextPoint[0]:
        DistX = prevPoint[0] - nextPoint[0]
    else:
        DistX = nextPoint[0] - prevPoint[0]

    if prevPoint[1] >= nextPoint[1]:
        DistY = prevPoint[1] - nextPoint[1]
    else:
        DistY = nextPoint[1] - prevPoint[1]

    if DistX == 0:
        tan= 0
    else:
        tan = DistY/1.0/DistX
    angle = np.arctan(tan)
    dist = travelled - TotDist0
    x = np.cos(angle) * dist
    y = np.sin(angle) * dist

    if prevPoint[0] <= nextPoint[0]:
        carPoseX = prevPoint[0] + x
    else:
        carPoseX = prevPoint[0] - x

    if prevPoint[1] <= nextPoint[1]:
        carPoseY = prevPoint[1] + y
    else:
        carPoseY = prevPoint[1] - y

    return [[carPoseX, carPoseY], prevPoint, nextPoint]

def AddVisibleCones(name2, xy, carPose, carAngle, conesData):
    Index = []
    distance = []
    for cone in range(len(xy)):
        dist = DistFinder(carPose, xy[cone])
        if dist < 10:
            NewAngle = AngleChange(CarPose, xy[cone], carAngle)
            if NewAngle < 60 and NewAngle > -60:
                Index.append(cone)
                distance.append(dist)

    sorted = np.argsort(distance)
    Index = np.array(Index)
    Index = Index[sorted]

    for cone in Index:
        check = False
        file = open(name2 + '.csv', 'r')
        for line in file:
            if line == conesData[cone] :
                check = True
        if check == False:
            file.close()
            file = open(name2 +'.csv', 'a')
            file.write(conesData[cone])
            file.close()
        else:
            file.close()

def CarOrientation(nextPoint, prevPoint):

    PointShifted = [nextPoint[0] - prevPoint[0], nextPoint[1] - prevPoint[1]]
    PointToCar = DistFinder([0, 0], PointShifted)
    cosPoint = PointShifted[0] / 1.0/ PointToCar
    PointAngle = np.arccos(cosPoint)
    PointAngle = PointAngle * 180/ math.pi
    if PointShifted[1] < 0:
        PointAngle = PointAngle * -1    

    AngleToCar = 90 - PointAngle
    if AngleToCar > 180:
        AngleToCar = AngleToCar - 360
    elif AngleToCar < -180:
        AngleToCar = AngleToCar + 360
    
    return AngleToCar

# Find if two segments intersect
# if D1 is a segment between A and B, D1 = [[xA, yA],[xB, yB]]
def IsIntersect(D1,D2):  
    D1 = np.array(D1)
    D2 = np.array(D2)

    # Define the range of each segment in the X-axis and Y-axis
    # So that RangeXD1[0] contains the smallest X coordinate of D1 and RangeXD1[1] the biggest
    if D1[0,0] <= D1[1,0]:
        RangeXD1 = [D1[0,0],D1[1,0]]
    else:
        RangeXD1 = [D1[1,0],D1[0,0]]

    if D1[0,1] <= D1[1,1]:
        RangeYD1 = [D1[0,1],D1[1,1]]
    else:
        RangeYD1 = [D1[1,1],D1[0,1]]

    if D2[0,0] <= D2[1,0]:
        RangeXD2 = [D2[0,0],D2[1,0]]
    else:
        RangeXD2 = [D2[1,0],D2[0,0]]

    if D2[0,1] <= D2[1,1]:
        RangeYD2 = [D2[0,1],D2[1,1]]
    else:
        RangeYD2 = [D2[1,1],D2[0,1]]

    # special case, if the two segment are perpendiculaire to the X-axis
    if (D1[0,0] == D1[1,0]) and (D2[0,0] == D2[1,0]):

        # special case, if the two segment are perpendiculaire to the X-axis and aligned
        if D1[0,0] == D2[0,0]:
            if ((RangeYD1[0] <= RangeYD2[1]) and (RangeYD1[0] >= RangeYD2[0]))  or ((RangeYD1[1] >= RangeYD2[0]) and (RangeYD1[1] <= RangeYD2[1])) :
                intersect = True
            else: 
                intersect = False
        else:
            intersect = False

    # special case, if the two segment are parrallel to the X-axis
    elif (D1[0,1] == D1[1,1]) and (D2[0,1] == D2[1,1]):

        # special case, if the two segment are parrallel to the X-axis and aligned
        if D1[0,1] == D2[0,1]:
            if ((RangeXD1[0] <= RangeXD2[1]) and (RangeXD1[0] >= RangeXD2[0]))  or ((RangeXD1[1] >= RangeXD2[0]) and (RangeXD1[1] <= RangeXD2[1])) :
                intersect = True
            else: 
                intersect = False
        else:
            intersect = False  

    # Find CrossX and CrossY where the segment would cross if they were infinite
    else:
        # special case, if D1 is perpendicular to the X-axis     
        if D1[0,0] == D1[1,0]:
            CrossX= D1[0,0]

            # if D1 is perpendicular to the X-axis and D2 is parallel to the X - axis  
            if D2[0,1] == D2[1,1]:
                CrossY= D2[0,1] 
            
            # if D1 is perpendicular to the X-axis and D2 is defined by Y = c * X + d
            else:
                c = (D2[0,1] - D2[1,1])/1.0/(D2[0,0] - D2[1,0])
                d = D2[0,1]- (D2[0,0]*c)
                CrossY = c * CrossX + d

        # special case, if D2 is perpendicular to the X-axis
        elif D2[0,0] == D2[1,0]:
            CrossX= D2[0,0] 

            # if D2 is perpendicular to the X-axis and D1 is parallel to the X - axis
            if D1[0,1] == D1[1,1]:
                CrossY= D1[0,1] 

            # if D2 is perpendicular to the X-axis and D1 is defined by Y = a * X + b  
            else:
                a = (D1[0,1] - D1[1,1])/1.0/(D1[0,0] - D1[1,0])
                b = D1[0,1]- (D1[0,0]*a)
                CrossY = a * CrossX + b

        # special case, if D1 is parallel to the X-axis 
        elif D1[0,1] == D1[1,1]:
            CrossY= D1[0,1]

            # if D1 is parrallel to the X-axis and D2 is perpendicular to the X - axis  
            if D2[0,0] == D2[1,0]:
                CrossX= D2[0,0] 

            # if D1 is parrallel to the X-axis and D2 is defined by Y = c * X + d   
            else:
                c = (D2[0,1] - D2[1,1])/1.0/(D2[0,0] - D2[1,0])
                d = D2[0,1]- (D2[0,0]*c)
                CrossX = (CrossY - d)/c

        # special case, if D2 is parallel to the X-axis 
        elif D2[0,1] == D2[1,1]:
            CrossY= D2[0,1]

            # if D2 is parrallel to the X-axis and D1 is perpendicular to the X - axis
            if D1[0,0] == D1[1,0]:
                CrossX= D2[0,0] 

            # if D2 is parrallel to the X-axis and D1 is defined by Y = a * X + b  
            else:
                a = (D1[0,1] - D1[1,1])/1.0/(D1[0,0] - D1[1,0])
                b = D1[0,1]- (D1[0,0]*a)
                CrossX = (CrossY - b)/a

        # General case, where D1 is defined by Y = a * X + b and D2 by Y = c * X + d
        else:
            a = (D1[0,1] - D1[1,1])/1.0/(D1[0,0] - D1[1,0])
            b = D1[0,1]- (D1[0,0]*a)

            c = (D2[0,1] - D2[1,1])/1.0/(D2[0,0] - D2[1,0])
            d = D2[0,1]- (D2[0,0]*c)

            # Special case where, D1 and D2 are parallel but not parallel or perpendicular to an axis
            if a == c:
                # if not aligned
                if b != d:
                    intersect = False
                # if aligned
                elif ((RangeYD1[0] <= RangeYD2[1]) and (RangeYD1[0] >= RangeYD2[0]))  or ((RangeYD1[1] >= RangeYD2[0]) and (RangeYD1[1] <= RangeYD2[1])) :
                    intersect = True
                else:
                    intersect = False
                return intersect
                
            CrossX = (d - b) /1.0/ (a - c)
            CrossY = (c*b - a*d)/1.0/(c - a)

        # If CrossX and CrossY are included in the x - axis and y - axis range of D1 and D2, then they intersect 
        if (CrossX >= RangeXD1[0] and CrossX <= RangeXD1[1]) and (CrossX >= RangeXD2[0] and CrossX <= RangeXD2[1]):
            inX = True
        else:
            inX = False

        if (CrossY >= RangeYD1[0] and CrossY <= RangeYD1[1]) and (CrossY >= RangeYD2[0] and CrossY <= RangeYD2[1]):
            inY = True
        else:
            inY = False

        if inX == True and inY == True:
            intersect = True
        else:
            intersect = False

    return(intersect)

Name = 'Full_SLAM_Input'
Name2 = 'Partial_SLAM_Input'
ClearFile(Name2)
ConesData = ReadFile(Name)
ConesXY, Color = Separator(ConesData)

StartPoint = [0,0] 
CarPose = StartPoint
CarAngle = 90 #CarStartAngle
Speed = 1
OneLap = True
dist = 0
DistToNext = 0
FirstPoint = [0,0]
TimeRecord = []

AddVisibleCones(Name2, ConesXY, CarPose, CarAngle, ConesData)
StartT= time.time()
Trajectory= LocalPlanning3_0.Main(CarPose, CarAngle, FirstPoint)
ComputeTime = time.time() - StartT
FirstPoint = Trajectory[1]
t0 = time.time()
DistToNext = DistFinder(Trajectory[0], Trajectory[1])

i = 0
while OneLap == True: 
    if i == 30:
        print(CarPose)
        print(CarAngle)
        exit()
    t = time.time() - t0
    CarPose, PrevPoint, NextPoint = CarPosition(Speed, t, Trajectory)
    AddVisibleCones(Name2, ConesXY, CarPose, CarAngle, ConesData)
    t = time.time() - t0
    Travelled = t * Speed
    if Travelled > DistToNext: 
        print("i  ", i)
        t = time.time() - t0
        CarPose, PrevPoint, NextPoint = CarPosition(Speed, t, Trajectory)
        t0 = time.time()
        CarAngle = CarOrientation(NextPoint, PrevPoint)

       # print("CarPose ", CarPose)
       # print("CarAngle ", CarAngle)

        print(i)
        StartT= time.time()
        Trajectory = LocalPlanning3_0.Main(CarPose, CarAngle, FirstPoint)
        ComputeTime = time.time() - StartT

        #print("Len Traj  ", len(Trajectory))

        TimeRecord.append(ComputeTime)
        DistToEnd = 0
        for n in range(len(Trajectory) - 1):
            DistToEnd = DistToEnd + DistFinder(Trajectory[n], Trajectory[n +1])

        if ComputeTime * Speed > DistToEnd:
            print("ToSlow")
            print("DistToEnd")
            print(DistToEnd)
            print("Travelled")
            print(ComputeTime * Speed)
            print[TimeRecord]

            print(Trajectory)
            print(CarPose)
            print(CarAngle)
            exit() 

        S1 = [Trajectory[0], Trajectory[1]]
        S2 = [[0,-2],[0, 2]]
        if  IsIntersect(S1, S2) == True:
            OneLap == False
            print("One Lap")
            print(Trajectory)
            print(TimeRecord)
            
            Tot = 0
            for j in range(len(TimeRecord)):
                Tot = Tot + TimeRecord[i]    
            Average = Tot/1.0/ len(TimeRecord)
            print(Average)
            print(max(TimeRecord))

            exit()
        elif i > 200:
            print("exit")
            print(TimeRecord)
            exit()
        
        DistToNext = DistFinder(Trajectory[0], Trajectory[1])
        i = i+1