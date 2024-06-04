from turtle import * 
import numpy as np
import turtle

#Obstacle coordinates (x, y, radius)
obstacles =  [  [0, 0, 20] ]  
#Between X: -500, 500 & Y: -300, 300

def Vector(x1,y1,x2,y2): #Computes data of a given vector
    #Mathematic operations
    Dx = x2 - x1
    Dy = y2 - y1
    vectorCoord = [[Dx], [Dy]]
    vectorMagnitude = sqrt(Dx ** 2 + Dy ** 2)
    unitaryVector = [[Dx/vectorMagnitude],[Dy/vectorMagnitude]]
    angle = np.arctan2(Dy,Dx) * 57.29577

    return vectorCoord, vectorMagnitude, unitaryVector, angle 

def DrawVector(x1,y1,x2,y2): #Creates a visible vector on given coordinates
    #Mathematic operations
    angle = np.arctan2(y2 - y1, x2 - x1) * 57.29577

    #Vector line
    turtle.penup()
    turtle.goto(x1,y1)
    turtle.pendown()
    turtle.setheading(angle)
    turtle.forward(40)
    #Vector direction (Draws the head)
    turtle.right(210) 
    turtle.forward(5)
    turtle.backward(5)
    turtle.setheading(angle)
    turtle.left(210) 
    turtle.forward(5)

def DrawDestiny(xCoord, yCoord, side):
    turtle.penup()
    turtle.goto(xCoord, yCoord)

    pendown()
    turtle.forward(side)  
    turtle.left(120)
    turtle.forward(side)
    
    turtle.left(120)
    turtle.forward(side)

def DrawObstacle(obstacleList, obstacleQuantity):
    for i in range(obstacleQuantity):
        turtle.penup()
        turtle.goto(obstacleList[i][0], obstacleList[i][1])
        turtle.pendown()
        turtle.circle(obstacleList[i][2])

def AttractiveForce(xPosition, yPosition, xDestiny, yDestiny, Epsilon):
    xAttractiveForce = Epsilon * (xPosition - xDestiny)
    yAttractiveForce = Epsilon * (yPosition - yDestiny)
    
    return xAttractiveForce, yAttractiveForce

def RepulsiveForce(xDestiny, yDestiny, obstacleList, obstacleQuantity):
    xRepForce = 0
    yRepForce = 0

    for k in range(obstacleQuantity):
        Dq = [float(xDestiny - obstacleList[k][0]), float(yDestiny - obstacleList[k][1])]
        IDqI = sqrt(Dq[0] ** 2 + Dq[1] ** 2)
        Dq_u = [Dq[0]/IDqI, Dq[1]/IDqI]

        print("IDqI:")
        print(IDqI)

        #Define
        etha = 100000
        d0 = 5.0

        if(IDqI == 0):
            IDqI = 0.00001
        
        xRepulsive = etha * (1/IDqI - 1/d0) * (1/(IDqI**2)) * Dq_u[0] #X
        yRepulsive = etha * (1/IDqI - 1/d0) * (1/(IDqI**2)) * Dq_u[1] #Y
        print("C")

        xRepForce += xRepulsive
        yRepForce += yRepulsive
        print("Xrep: "); print(xRepulsive); print("Yrep: "); print(yRepulsive)

    return xRepForce, yRepForce

def GraphField(xSize, ySize, xDestiny, yDestiny, epsilon, space):
    xCicles = int(xSize / space)
    yCicles = int(ySize / space)
    
    xSize = -xSize/2
    ySize = -ySize/2

    for i in range(xCicles + 1):
        for j in range(yCicles + 1):
            xAttractive, yAttractive = AttractiveForce(xSize, ySize, xDestiny, yDestiny, epsilon)
            xRepulsive, yRepulsive = RepulsiveForce(xDestiny, yDestiny, obstacles, 1) #
            xDir = xRepulsive
            yDir = yRepulsive
            DrawVector(xSize + i * space, ySize + j *space, -xDir, -yDir)

def main():
    #Setup data
    xScreenSize = 500
    yScreenSize = 500
    #Destiny
    xObjective = 10
    yObjective = 10
    sizeDestiny = 20
    #Constants
    Epsilon = -0.001

    obj = turtle.Turtle()
    screen = Screen()
    screen.setup(xScreenSize, yScreenSize)

    DrawDestiny(xObjective, yObjective, sizeDestiny)
    DrawObstacle(obstacles, 1) #
    
    GraphField(xScreenSize, yScreenSize, xObjective, yObjective, Epsilon, 100)
    

    penup()
    goto(1000, 1000)
    turtle.done()

if __name__ == "__main__":
    main()


"""
def DirectionVector():
    pass

def 

    
"""






