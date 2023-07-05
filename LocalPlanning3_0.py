import numpy as np
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import random
import math

# Read the text file where information about cones that have been seen are stored
# Cones data is stored in the following format [X coordinate, Y coordinate, color]
# conesPoses stores the X and Y coordinates, conesColors stores the colors
# Both array have the same length and order, so that color and coordinates of a cone can be found with the same index
# Cones that too far from the car and behind teh acr are not added
def TextToArray(name1, carXY, carAngle):
    conesPoses = []
    conesColors = []
    file = open(name1 +'.csv', 'r')
    for line in file:
        if "blue" in line or "orange" in line or "yellow" in line:
            if line != "" and line != " " and line != '\n':
                coma1 = line.find(',')
                Color = line[0: coma1]
                coma2 = line[coma1+1:].find(',')
                coneX = float(line[coma1 + 1: coma1 + coma2 + 1])
                coma3 = line.find('\n')
                coneY = float(line[coma1 + coma2 + 2: coma3])
                conePose = []
                conePose.append(coneX)
                conePose.append(coneY)
                dist = DistFinder(conePose, carXY)
                if dist <= 15:
                    #if AngleChange(carXY, conePose, carAngle)< 90 and AngleChange(carXY, conePose, carAngle) >-90:
                    conesColors.append(Color)
                    conesPoses.append(conePose)
    file.close()
    conesPoses = np.array(conesPoses)
    return [conesPoses, conesColors]


# Find the coordinate of the center of the edge of the Delauney simplices
# Only the coordinates of centers between a blue and a yellow cones are stored, meaning that the point is in the track
def MidPointFinder(conesPoses, conesColors, tri):
    MidPoints = []
    for i in range(len(tri.simplices)):
        for n in range(3):
            # if tri.neighbors = -1, the edge has no neighbor, therefore the segment is an external border of the track
            if tri.neighbors[i,n] == -1 or tri.neighbors[i,n] > i:
                #triangle contain the coordinates of the 3 vertices of a delauney triangle
                triangle = conesPoses[tri.simplices[i]]
                MidPointXY = [] 
                ColorA= ""
                ColorB= ""
                if n == 0:
                    for j in range(len(conesPoses)):
                        if conesPoses[j,0] == triangle[1,0] and conesPoses[j,1] == triangle[1,1]:
                            ColorA = conesColors[j]
                            break
                    for j in range(len(conesPoses)):
                        if conesPoses[j,0] == triangle[2,0] and conesPoses[j,1] == triangle[2,1]:
                            ColorB = conesColors[j]
                            break
                    # Check if the midpoint is on the borders or in the tracks
                    if (ColorA == 'blue' and ColorB =='yellow') or (ColorA == 'yellow' and ColorB =='blue'):
                            MidPointX = (triangle[1,0] + triangle[2,0])/2.
                            MidPointY = (triangle[1,1] + triangle[2,1])/2.
                if n == 1:
                    for j in range(len(conesPoses)):
                        if conesPoses[j,0] == triangle[0,0] and conesPoses[j,1] == triangle[0,1]:
                            ColorA = conesColors[j]
                            break
                    for j in range(len(conesPoses)):
                        if conesPoses[j,0] == triangle[2,0] and conesPoses[j,1] == triangle[2,1]:
                            ColorB = conesColors[j]
                            break
                    if (ColorA == 'blue' and ColorB =='yellow') or (ColorA == 'yellow' and ColorB =='blue'):
                        MidPointX = (triangle[0,0] + triangle[2,0])/2.
                        MidPointY = (triangle[0,1] + triangle[2,1])/2.
                if n == 2:
                    for j in range(len(conesPoses)):
                        if conesPoses[j,0] == triangle[0,0] and conesPoses[j,1] == triangle[0,1]:
                            ColorA = conesColors[j]
                            break
                    for j in range(len(conesPoses)):
                        if conesPoses[j,0] == triangle[1,0] and conesPoses[j,1] == triangle[1,1]:
                            ColorB = conesColors[j]
                            break
                    if (ColorA == 'blue' and ColorB =='yellow') or (ColorA == 'yellow' and ColorB =='blue'):
                        MidPointX = (triangle[0,0] + triangle[1,0])/2.
                        MidPointY = (triangle[0,1] + triangle[1,1])/2.

                if ColorA != ColorB:
                    MidPointXY.append(MidPointX)
                    MidPointXY.append(MidPointY)
                    MidPoints.append(MidPointXY)
    MidPoints = np.array(MidPoints)
    return MidPoints

# Find the absolute distance between two points 
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


# Find if two segments intersect
# if S1 is a segment between A and B, S1 = [[xA, yA],[xB, yB]]
def IsIntersect(S1,S2):  
    S1 = np.array(S1)
    S2 = np.array(S2)

    # Define the range of each segment in the X-axis and Y-axis
    # So that RangeXD1[0] contains the smallest X coordinate of S1 and RangeXD1[1] the biggest
    if S1[0,0] <= S1[1,0]:
        RangeXD1 = [S1[0,0],S1[1,0]]
    else:
        RangeXD1 = [S1[1,0],S1[0,0]]

    if S1[0,1] <= S1[1,1]:
        RangeYD1 = [S1[0,1],S1[1,1]]
    else:
        RangeYD1 = [S1[1,1],S1[0,1]]

    if S2[0,0] <= S2[1,0]:
        RangeXD2 = [S2[0,0],S2[1,0]]
    else:
        RangeXD2 = [S2[1,0],S2[0,0]]

    if S2[0,1] <= S2[1,1]:
        RangeYD2 = [S2[0,1],S2[1,1]]
    else:
        RangeYD2 = [S2[1,1],S2[0,1]]

    # special case, if the two segment are perpendiculaire to the X-axis
    if (S1[0,0] == S1[1,0]) and (S2[0,0] == S2[1,0]):

        # special case, if the two segment are perpendiculaire to the X-axis and aligned
        if S1[0,0] == S2[0,0]:
            if ((RangeYD1[0] <= RangeYD2[1]) and (RangeYD1[0] >= RangeYD2[0]))  or ((RangeYD1[1] >= RangeYD2[0]) and (RangeYD1[1] <= RangeYD2[1])) :
                intersect = True
            else: 
                intersect = False
        else:
            intersect = False

    # special case, if the two segment are parrallel to the X-axis
    elif (S1[0,1] == S1[1,1]) and (S2[0,1] == S2[1,1]):

        # special case, if the two segment are parrallel to the X-axis and aligned
        if S1[0,1] == S2[0,1]:
            if ((RangeXD1[0] <= RangeXD2[1]) and (RangeXD1[0] >= RangeXD2[0]))  or ((RangeXD1[1] >= RangeXD2[0]) and (RangeXD1[1] <= RangeXD2[1])) :
                intersect = True
            else: 
                intersect = False
        else:
            intersect = False  

    # Find CrossX and CrossY where the segment would cross if they were infinite
    else:
        # special case, if S1 is perpendicular to the X-axis     
        if S1[0,0] == S1[1,0]:
            CrossX= S1[0,0]

            # if S1 is perpendicular to the X-axis and S2 is parallel to the X - axis  
            if S2[0,1] == S2[1,1]:
                CrossY= S2[0,1] 
            
            # if S1 is perpendicular to the X-axis and S2 is defined by Y = c * X + d
            else:
                c = (S2[0,1] - S2[1,1])/1.0/(S2[0,0] - S2[1,0])
                d = S2[0,1]- (S2[0,0]*c)
                CrossY = c * CrossX + d

        # special case, if S2 is perpendicular to the X-axis
        elif S2[0,0] == S2[1,0]:
            CrossX= S2[0,0] 

            # if S2 is perpendicular to the X-axis and S1 is parallel to the X - axis
            if S1[0,1] == S1[1,1]:
                CrossY= S1[0,1] 

            # if S2 is perpendicular to the X-axis and S1 is defined by Y = a * X + b  
            else:
                a = (S1[0,1] - S1[1,1])/1.0/(S1[0,0] - S1[1,0])
                b = S1[0,1]- (S1[0,0]*a)
                CrossY = a * CrossX + b

        # special case, if S1 is parallel to the X-axis 
        elif S1[0,1] == S1[1,1]:
            CrossY= S1[0,1]

            # if S1 is parrallel to the X-axis and S2 is perpendicular to the X - axis  
            if S2[0,0] == S2[1,0]:
                CrossX= S2[0,0] 

            # if S1 is parrallel to the X-axis and S2 is defined by Y = c * X + d   
            else:
                c = (S2[0,1] - S2[1,1])/1.0/(S2[0,0] - S2[1,0])
                d = S2[0,1]- (S2[0,0]*c)
                CrossX = (CrossY - d)/c

        # special case, if S2 is parallel to the X-axis 
        elif S2[0,1] == S2[1,1]:
            CrossY= S2[0,1]

            # if S2 is parrallel to the X-axis and S1 is perpendicular to the X - axis
            if S1[0,0] == S1[1,0]:
                CrossX= S2[0,0] 

            # if S2 is parrallel to the X-axis and S1 is defined by Y = a * X + b  
            else:
                a = (S1[0,1] - S1[1,1])/1.0/(S1[0,0] - S1[1,0])
                b = S1[0,1]- (S1[0,0]*a)
                CrossX = (CrossY - b)/a

        # General case, where S1 is defined by Y = a * X + b and S2 by Y = c * X + d
        else:
            a = (S1[0,1] - S1[1,1])/1.0/(S1[0,0] - S1[1,0])
            b = S1[0,1]- (S1[0,0]*a)

            c = (S2[0,1] - S2[1,1])/1.0/(S2[0,0] - S2[1,0])
            d = S2[0,1]- (S2[0,0]*c)

            # Special case where, S1 and S2 are parallel but not parallel or perpendicular to an axis
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

        # If CrossX and CrossY are included in the x - axis and y - axis range of S1 and S2, then they intersect 
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

# Create 2 lists containning the coordinates of all Yellow cones and all Blue cones 
def Borders(conesPoses, conesColors):
    BorderY = []
    BorderB = []
    for i in range(len(conesPoses)):
        if conesColors[i] == "yellow":
            BorderY.append(conesPoses[i])
        elif conesColors[i] == "blue":
            BorderB.append(conesPoses[i])
    return [BorderY, BorderB]

# Check if two possible trajectory points can be linked without intersercting one of the border
def IsCrossBorder(ClosestPoint, NewPoint, conesPose, conesColor):
    S1 = [[0,0],[0,0]]
    S2 = []
    S2.append(ClosestPoint)
    S2.append(NewPoint)
    Yellow = Borders(conesPose, conesColor)[0]
    Blue = Borders(conesPose, conesColor)[1]

    for i in (range(len(Yellow))):
        S1[0] = Yellow[i]
        for n in range(len(Yellow)):
            S1[1] = Yellow[n]
            dist = DistFinder(S1[0], S1[1])
            if dist <= 5:
                if IsIntersect(S1, S2) == True:
                    return True

    for i in (range(len(Blue))):
        S1[0] = Blue[i]
        for n in range(len(Blue)):
            S1[1] = Blue[n]
            dist = DistFinder(S1[0], S1[1])
            if dist <= 5:
                if IsIntersect(S1, S2) == True:
                    return True
    
    return False

# Find what is the angle between the car and the next point based on the car orientation
# A point on the positive Y -axis is at 0 degree, a point on the negative Y -axis is at 180 or -180 degree
# A point on the positive X -axis is at 90 degree, a point on the negative Y -axis is at -90 degree
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

# Build a path through the centers point, starting from the car position
# The next point as to be at less than 10 meter, in a range of 180 degree based on the car orientation, not intersect a border.
# Among the possible points, the closest is selected 
def PathBuilder(carXY, centers, conesPoses, conesColors, carAngle, Start):
    trajPoints = [carXY]
    i = 0
    NbPossiblePts = 1
    while NbPossiblePts != 0:
        PossiblePts = []
        for n in range(len(centers)):
            dist = DistFinder(trajPoints[i], centers[n])
            #Check if the point is at less than 10m from the car. Further cones have more uncetainty on their position. Close cones are considered for safety
            if dist > 0:
                if len(trajPoints) == 1:
                    PrevAngle = carAngle
                    PrevPoint = carXY
                # Find the deviation between the last point of the path and the next one
                NewAngle = AngleChange(PrevPoint, centers[n], PrevAngle)
                #Check if the next point is in front of the current one based on the orientation of the car if it follows the path.
                #Allows to avoid going back on points already contained in the path
                if NewAngle < 90 and NewAngle > -90:
                    # Check if addding the next point to the trajectory would cross a border
                    if IsCrossBorder(PrevPoint, centers[n], conesPoses, conesColors) == False:
                        PossiblePts.append(centers[n])
        NbPossiblePts = len(PossiblePts)
        #Stop finding new points if no points match the criteria
        if NbPossiblePts == 0:
            break
        # Among the possible points select the closest one 
        else:
            distMin = 1000
            for n in range(NbPossiblePts):
                dist = DistFinder(trajPoints[i],PossiblePts[n])
                if dist < distMin:
                    distMin = dist
                    distMinIndex = n  
            trajPoints.append(PossiblePts[distMinIndex])

            # update variables
            i = i+1
            PrevAngle = AngleChange(PrevPoint, PossiblePts[distMinIndex], PrevAngle) + PrevAngle
            if PrevAngle >= 360:
                PrevAngle = PrevAngle - 360
            elif PrevAngle <= -360:
                PrevAngle = PrevAngle + 360
            PrevPoint = PossiblePts[distMinIndex]
        
        # Stop the loop if the start point of the track is added to the path
        # This means that the path found has reached the start line again. at that point the Global Planning will continue
        S1 = [trajPoints[len(trajPoints) - 2], PossiblePts[distMinIndex]]
        S2 = [[0,-2],[0, 2]]
        if  IsIntersect(S1, S2) == True and i !=1:
            print("Traj reach lap")
            break
        # Protect the programm if blocked in an infinite loop
        elif i>200:
            print('exit2')
            break

    return trajPoints


def ClearFile(name2):
    file = open(name2 +'.csv', 'w')
    file.truncate()
    file.close()

def WriteOutput(Traj, borderY, borderB, name2):
    file = open(name2 +'.csv', 'a')
    file.write("tag,x,y\n")

    for i in range(len(Traj)):
        file.write("point," + str(Traj[i][0]) + "," + str(Traj[i][1]) + "\n")
    for i in range(len(borderY)):
        file.write("yellow," + str(borderY[i][0]) + "," + str(borderY[i][1]) + "\n")
    for i in range(len(borderB)):
        file.write("blue," + str(borderB[i][0]) + "," + str(borderB[i][1]) + "\n")
    
    file.close()

def Main(carPose, CarAngle, firstPoint):

    InputFile = "Partial_SLAM_Input"
    OutputFile = "Control_Output"

    # Get cones coordinates and color from a text file from SLAM
    cones = TextToArray(InputFile, carPose, CarAngle)
    conesXY = cones[0]
    Color = cones[1]

    #Discretise the environment with Delauney triangulation method
    Tri = Delaunay(conesXY)

    # Find the centers of Delauney triangles' edges and keep the one inside the track 
    Centers = MidPointFinder(conesXY,Color, Tri)


    # Create array containning coordinates of borders
    BorderY, BorderB = Borders(conesXY, Color)
    BorderY = np.array(BorderY)
    BorderB = np.array(BorderB)


    # Find the trajectory to stay inside the track starting from the car position
    Trajectory = PathBuilder(carPose, Centers, conesXY, Color, CarAngle, firstPoint)
    Trajectory = np.array(Trajectory)

    ClearFile(OutputFile)
    WriteOutput(Trajectory, BorderY, BorderB, OutputFile)


    """
    # Plot the Delauney triangulation
    plt.triplot(conesXY[:,0], conesXY[:,1], Tri.simplices)
    #plt.plot(conesXY[:,0], conesXY[:,1], 'o')
    # Plot the track left border in yellow
    plt.plot(BorderY[:,0], BorderY[:,1], 'yo')
    # Plot the track right border in blue
    plt.plot(BorderB[:,0], BorderB[:,1], 'bo')
    # Plot the centers point in green 
    plt.plot(Centers[:,0], Centers[:,1], 'go')
    # Plot and link the trajectory point in red
    plt.plot(Trajectory[:,0], Trajectory[:,1], marker = 'o', color = 'red')

    plt.show()
    """

    return Trajectory
    
 