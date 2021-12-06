import math

# Turn the robot to certain degree
def turnRobot(current, des):
    if (des > current):
        while current != des:
            current += 5
    elif (des < current):
        while current != des:
            current -= 5
    return current

def moveOnPath(robot, path):
    for i in path:
        if i == 0:
            i = 1
        orientation = getAngle(robot[0], path[i])
        robot[1] = turnRobot(robot[1], orientation)
        robot[0] = path[i]

moveOnPath(robot, path)
RemovePizza() #Olzhas remove pizza
path = path.reverse()
robot[1] = turnRobot(robot[1], robot[1] + 180)
moveOnPath(robot,path)