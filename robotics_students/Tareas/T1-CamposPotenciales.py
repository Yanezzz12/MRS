from turtle import * 
import numpy as np
import turtle

def vector(x1,y1,x2,y2):
    #Mathematic operations
    Dx = x2 - x1
    Dy = y2 - y1
    vectorCoord = [[Dx], [Dy]]
    vectorMagnitude = sqrt(Dx ** 2 + Dy ** 2)
    unitaryVector = [[Dx/vectorMagnitude],[Dy/vectorMagnitude]]
    angle = np.arctan2(Dy,Dx) * 57.29577

    #Vector line
    turtle.penup()
    turtle.goto(x1,y1)
    turtle.pendown()
    turtle.setheading(angle)
    turtle.forward(50)
    #Vector direction
    turtle.right(210) 
    turtle.forward(10)
    turtle.backward(10)
    turtle.setheading(angle)
    turtle.left(210) 
    turtle.forward(10)
    #Reset vector
    turtle.penup()
    turtle.home()

    return vectorCoord, vectorMagnitude, unitaryVector, angle 

def FieldEquation(position, destiny, obstacle):
    Epsilon = 1
    Etha = 1
    
    d0 = 1
    Uatr = [(0.5 * Epsilon) * abs(position[0]-destiny[0]) ** 2, (0.5 * Epsilon) * abs(position[1]-destiny[1]) ** 2]
    Urep = [0,0]
    #Urep = [(0.5 * Etha) * (1/(position[0] - obstacle[0]) - 1/d0) ** 2, (0.5 * Etha) * (1/(position[1] - obstacle[1]) - 1/d0) ** 2,]

    Utot = [Uatr[0] + Urep[0], Uatr[1] + Urep[1]]

    print(Uatr)
    print(Urep)
    print(Utot)
    return Utot[0], Utot[1]

def GraphicField():
    xRange = 300
    yRange = 300

    step = 100

    
    for i in range((2*xRange)/step)
        for j in range((2*yRange)/step)
            




def main():
    obj = turtle.Turtle()
    screen = Screen()
    screen.setup(2000, 1200)


    FieldEquation([50, 50], [100, 143], [220,20])


    #turtle.done()

if __name__ == "__main__":
    main()












