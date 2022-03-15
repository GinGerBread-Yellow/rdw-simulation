
import math
from random import randint


def generatePath(length, section):
    assert(length > 0 and section > 0)
    cood = []
    for i in range(section+1):
        y = round(math.sqrt(section**2-i**2))
        cood.append(y)
    
    x,y = 0,0
    path = [(x,y)]
    for i in range(length-1):
        dx = randint(-section, section)
        dy = cood[abs(dx)]
        dir = randint(0,1)
        if dir > 0:
            dy = -dy
        x, y = x+dx, y+dy
        path.append((x,y))
    return path

    
