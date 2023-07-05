# Theodore Bedos 28/03/21
# Script to plot image of measured temperature, and trace it using the mouse.
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt


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




#Create a variable for the image
name = "TrackAndCones2"
img= Image.open(name + '.png')
imgWidth, imgHeight  = img.size
pix = list(img.getdata())
pix = np.array(pix).reshape(imgHeight, imgWidth, 4)
type = img.mode

print(imgWidth)
print(imgHeight)


yaxisY = []
xaxisY = []
yaxisB = []
xaxisB = []
yaxisG = []
xaxisG = []
yaxisW = []
xaxisW = []


#Filter all the black pixels on the graph
for i in range(imgHeight):
    for n in range(imgWidth):
        if pix[i][n][0]>180 and pix[i][n][1]>180 and pix[i][n][2]<80:
            yaxisY.append(i)
            xaxisY.append(n)
        elif pix[i][n][0]<60 and pix[i][n][1]<60 and pix[i][n][2]>180:
            yaxisB.append(i)
            xaxisB.append(n)
        elif pix[i][n][0]<50 and pix[i][n][1]>200 and pix[i][n][2]<50:
            yaxisG.append(i)
            xaxisG.append(n)
        elif pix[i][n][0]>200 and pix[i][n][1]>200 and pix[i][n][2]>200:
            yaxisW.append(i)
            xaxisW.append(n)


# [xaxis, index] = unique(xaxis, 'last');
# yaxis = yaxis(index);

# Define the length of the green line
ymax = max(yaxisG)
y0 = min(yaxisG)
Ylen = ymax- y0
print("Ylen")
print(Ylen)

#Define origine
xaxisW = np.array(xaxisW)
yaxisW = np.array(yaxisW)
print(round(xaxisW[len(xaxisW)/2]))
OrigineX = (round(xaxisW[len(xaxisW)/2]) * 5) / 1.0/ Ylen
OrigineY = ((-1 * yaxisW[len(yaxisW)/2] + imgHeight) * 5) / 1.0/Ylen

# Transform tules into arrays
xaxisY = np.array(xaxisY)
yaxisY = np.array(yaxisY)
xaxisB = np.array(xaxisB)
yaxisB = np.array(yaxisB)


yaxisY= ((-1 *yaxisY + imgHeight )*5/1.0/Ylen) - OrigineY
xaxisY=(xaxisY*5/1.0/Ylen) - OrigineX
yaxisB= ((-1 *yaxisB + imgHeight )*5/1.0/Ylen) - OrigineY
xaxisB=(xaxisB*5/1.0/Ylen) - OrigineX

i = 0
while i < len(yaxisY):
    n = 0
    while n < len(yaxisY):
        if i !=n :
            dist = DistFinder([xaxisY[i],yaxisY[i]], [xaxisY[n],yaxisY[n]])
            if dist < 1:
                xaxisY = np.delete(xaxisY, n)
                yaxisY = np.delete(yaxisY, n)
                n =n-1
        n = n+1
        if n>500:
            exit()
    i = i +1
    if i>500:
        exit()

i = 0
while i < len(yaxisB):
    n = 0
    while n < len(yaxisB):
        if i !=n :
            dist = DistFinder([xaxisB[i],yaxisB[i]], [xaxisB[n],yaxisB[n]])
            if dist < 1:
                xaxisB = np.delete(xaxisB, n)
                yaxisB = np.delete(yaxisB, n)
                n = n-1
        n = n+1
        if n>2000:
            print('exit')
            exit()
    i = i +1
    if i>2000:
        print('exit')
        exit()


file = open('Full_SLAM_Input.csv', 'w')
file.truncate()
file.write("tag,x,y\n")
for i in range(len(yaxisY)):
    line = 'yellow,' + str(xaxisY[i]) + ',' + str(yaxisY[i])
    file.write(line)
    file.write('\n')
for i in range(len(yaxisB)):
    line = 'blue,' + str(xaxisB[i]) + ',' + str(yaxisB[i])
    file.write(line)
    file.write('\n')
file.close()




plt.plot(xaxisY, yaxisY, 'yo')
plt.plot(xaxisB, yaxisB, 'bo')
plt.show()
