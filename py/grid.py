
import math
import numpy
import entity
import heapq

GRID_NEW_VALUE_WEIGHT   = 0.6
GRID_CIRCLE_GRANULARITY = 0.025
GRID_MAP_FRAME_SLICES   = 8
GRID_MAP_FRAME_STEP     = 0.25
GRID_MAP_ANGLE_SLICING  = 20
PI_1l2                  = 1.57079632679
PI_3l2                  = 4.71238898038
PI                      = 3.14159265358

def dist2(x1,y1,x2,y2):
    return numpy.ceil((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1))

class Frame:
    angle = 0
    weight = 0
    posX = 0
    posY = 0
    step = 0
    prev = None
    
    def get_id(self,g):
        ret = ((int(numpy.floor((180/PI) * self.angle) // GRID_MAP_ANGLE_SLICING)&0xff)<<47) 
        ret |= g.round_out_bottom(self.posX)<<23
        ret |= g.round_out_bottom(self.posY)
        return ret

    def __init__(self,angle,weight,posX,posY,step):
        self.angle = angle
        self.weight = weight
        self.posX = posX
        self.posY = posY
        self.step = step

    def __init__(self,old,enterAngle):
        self.angle = numpy.fmod(old.angle + enterAngle,PI)
        self.step = old.step + 1
        self.posX = old.posX + (numpy.cos(self.angle)*GRID_MAP_FRAME_STEP)
        self.posY = old.posY + (numpy.sin(self.angle)*GRID_MAP_FRAME_STEP)
        self.prev = old

    def __init__(self,ar1,ar2,ar3=None,ar4=None,ar5=None):
        if ar3 is None:
            self.angle = numpy.fmod(ar1.angle + ar2,PI)
            self.step = ar1.step + 1
            self.posX = ar1.posX + (numpy.cos(self.angle)*GRID_MAP_FRAME_STEP)
            self.posY = ar1.posY + (numpy.sin(self.angle)*GRID_MAP_FRAME_STEP)
            self.prev = ar1
        else:
            self.angle = ar1
            self.weight = ar2
            self.posX = ar3
            self.posY = ar4
            self.step = ar5

    def __lt__(self,other):
        return self.weight < other.weight

class Grid:
    entities = []
    unitSize = 0.0
    sizeX = 0
    sizeY = 0
    grid = numpy.zeros((0,0))
    marks = numpy.zeros((0,0))

    def round_out_top(self,val):
        return int(numpy.floor(val/self.unitSize))

    def round_out_bottom(self,val):
        return int(numpy.ceil(val/self.unitSize))
    
    def avg_weights(self,old,nev,newWeight):
        if newWeight > 1:
            print("Weight over 1.0! : ",newWeight)
            newWeight = 1
        elif newWeight < 0:
            print("Weight under 0.0! : ",newWeight)
            newWeight = 0
        return numpy.ceil((old * (1 - newWeight)) + (nev * newWeight))

    def update_grid(self,shape, danger, dist1, dist2, posX, posY, weight):
        if shape == entity.ENTITY_TYPE_RECT:
            x = self.round_out_bottom(posX)
            y = self.round_out_bottom(posY)
            width = self.round_out_top(dist1)
            height = self.round_out_top(dist2)
            x = int(x - (width/2))
            y = int(y - (height/2))
            for i in range(x,width+x+1):
                if i < 0 or i > self.sizeX:
                    continue
                for j in range(y,height+y+1):
                    if j < 0 or j > self.sizeY:
                        continue
                    self.grid[i,j] = self.avg_weights(self.grid[i,j],danger,weight)
        elif shape == entity.ENTITY_TYPE_POINT:
            x = self.round_out_bottom(posX)
            y = self.round_out_bottom(posY)
            if not (x < 0 or x > self.sizeX or y < 0 or y > self.sizeY):
                self.grid[x,y] = self.avg_weights(grid[x,y],danger,weight)
        elif shape ==  entity.ENTITY_TYPE_CIRCLE:
            rad = self.round_out_top(dist1)
            x = self.round_out_bottom(posX)
            y = self.round_out_bottom(posY)
            xGP = -100
            xLP = -100
            for k in numpy.arange(0.0,PI+GRID_CIRCLE_GRANULARITY,GRID_CIRCLE_GRANULARITY):
                xG = int(numpy.ceil(x + numpy.cos(k) * rad))
                xL = int(numpy.floor(x + numpy.cos(k+PI) * rad))
                yG = int(numpy.ceil(y + numpy.sin(k) * rad))
                yL = int(numpy.floor(y + numpy.sin(k+PI) * rad))
                if xG != xGP:
                    if xG < 0 or xG > self.sizeX:
                        continue
                    for i in range(yL,yG+1):
                        if i < 0 or i > self.sizeY:
                            continue
                        self.grid[xG,i] = self.avg_weights(self.grid[xG,i],danger,weight)
                if xL != xLP and xL != xG:
                    if xL < 0 or xL > self.sizeX:
                        continue
                    for i in range(yL,yG+1):
                        if i < 0 or i > self.sizeY:
                            continue
                        self.grid[xL,i] = self.avg_weights(self.grid[xL,i],danger,weight)
                xGP = xG
                xLP = xL

    def in_bounds(self,x,y):
        if x < 0 or y < 0 or x > (self.unitSize*(self.sizeX-1)) or y > (self.unitSize*(self.sizeY-1)):
            return False
        return True

    def get_cost(self,e,frame):
        ret = 0
        if e.shape == entity.ENTITY_TYPE_RECT:
            x = self.round_out_bottom(frame.posX)
            y = self.round_out_bottom(frame.posY)
            width = self.round_out_top(e.dist1)
            height = self.round_out_top(e.dist2)
            x = int(x - (width/2))
            y = int(y - (height/2))
            for i in range(x,width+x+1):
                if i < 0 or i > self.sizeX:
                    continue
                for j in range(y,height+y+1):
                    if j < 0 or j > self.sizeY:
                        continue
                    ret += self.grid[i,j]
        elif e.shape == entity.ENTITY_TYPE_POINT:
            x = self.round_out_bottom(frame.posX)
            y = self.round_out_bottom(frame.posY)
            if not (x < 0 or x > self.sizeX or y < 0 or y > self.sizeY):
                ret += self.grid[i,j]
        elif e.shape == entity.ENTITY_TYPE_CIRCLE:
            rad = self.round_out_top(e.dist1)
            x = self.round_out_bottom(frame.posX)
            y = self.round_out_bottom(frame.posY)
            xGP = -100
            xLP = -100
            for k in numpy.arange(0.0,PI+GRID_CIRCLE_GRANULARITY,GRID_CIRCLE_GRANULARITY):
                xG = numpy.ceil(x + numpy.cos(k) * rad)
                xL = numpy.floor(x + numpy.cos(k+PI) * rad)
                yG = numpy.ceil(y + numpy.sin(k) * rad)
                yL = numpy.floor(y + numpy.sin(k+PI) * rad)
                if xG != xGP:
                    if xG < 0 or xG > self.sizeX:
                        continue
                    for i in range(yL,yG+1):
                        if i < 0 or i > self.sizeY:
                            continue
                        ret += self.grid[xG,i]
                if xL != xLP and xL != xG:
                    if xL < 0 or xL > self.sizeX:
                        continue
                    for i in range(yL,yG+1):
                        if i < 0 or i > self.sizeY:
                            continue
                        ret += self.grid[xL,i]
                xGP = xG
                xLP = xL
        return ret

    def child_frame(self,old,e,step,destX,destY):
        nFrame = Frame(old,step)
        nFrame.weight = numpy.floor((nFrame.step*1) + (0.5 * dist2(nFrame.posX,nFrame.posY,destX,destY)) + self.get_cost(e,nFrame))
        return nFrame

    def add_entity(self,e):
        if (isinstance(e,entity.WeightedEntity)):
            self.entities.append(e)
            self.update_grid(e.shape,e.danger,e.dist1,e.dist2,e.posX,e.posY,e.weight)
        elif (isinstance(e,entity.Entity)):
            self.entities.append(e)
            self.update_grid(e.shape,e.danger,e.dist1,e.dist2,e.posX,e.posY,GRID_NEW_VALUE_WEIGHT)

    def print(self):
        for j in range(self.sizeY-1,0,-1):
            print("--",sep='',end='')
        print("---")
        for j in range(self.sizeY-1,-1,-1):
            for i in range(0,self.sizeX):
                if self.marks[i,j] == 1:
                    print("\033[43m",end='')
                elif self.marks[i,j] == 2:
                    print("\033[45m",end='')
                if self.grid[i,j] > 0x7f:
                    print("\033[36m",end='')
                elif self.grid[i,j] > 0x5f:
                    print("\033[35m",end='')
                elif self.grid[i,j] > 0x2f:
                    print("\033[34m",end='')
                elif self.grid[i,j] > 0x0f:
                    print("\033[31m",end='')
                elif self.grid[i,j] > 0x07:
                    print("\033[33m",end='')
                elif self.grid[i,j] > 0x00:
                    print("\033[32m",end='')
                print( "%2x\033[0m" % int(self.grid[i,j]),sep='',end='')
            print()
        for j in range(self.sizeY-1,0,-1):
            print("--",sep='',end='')
        print("---")

    def map_entity(self,entity,destX,destY):
        currPaths = []
        visited = {}

        destXG = self.round_out_top(destX)
        destYG = self.round_out_top(destY)
        destXL = self.round_out_bottom(destX)
        destYL = self.round_out_bottom(destY)

        start = Frame(entity.angle,0,entity.posX,entity.posY,0)
        heapq.heappush(currPaths,start)

        maxAChange = (PI/90) * (((numpy.arcsin((1/entity.turnRad)*(GRID_MAP_FRAME_STEP/2.0))*(180/PI))))
        aStep = maxAChange / GRID_MAP_FRAME_SLICES

        while currPaths:
            curr = heapq.heappop(currPaths)

            if ((self.round_out_top(curr.posX) == destXG or self.round_out_bottom(curr.posX) == destXL) and \
                (self.round_out_top(curr.posY) == destYG or self.round_out_bottom(curr.posY) == destYL)):
                ret = curr
                while curr != None:
                    self.marks[self.round_out_bottom(curr.posX),self.round_out_bottom(curr.posY)] = 2
                    curr = curr.prev
                self.print()
                self.marks = numpy.zeros((self.sizeX+1,self.sizeY+1))
                return ret

            # up left
            for i in range(0,GRID_MAP_FRAME_SLICES):
                nFrame = self.child_frame(curr,entity,(aStep*i),destX,destY)
                if (self.in_bounds(nFrame.posX,nFrame.posY) and not (nFrame.get_id(self) in visited)):
                    visited[nFrame.get_id(self)] = True
                    self.marks[self.round_out_bottom(nFrame.posX),self.round_out_bottom(nFrame.posY)] = 1
                    heapq.heappush(currPaths,nFrame)

            # up right
            for i in range(1,GRID_MAP_FRAME_SLICES):
                nFrame = self.child_frame(curr,entity,(aStep*i*-1),destX,destY)
                if (self.in_bounds(nFrame.posX,nFrame.posY) and not (nFrame.get_id(self) in visited)):
                    visited[nFrame.get_id(self)] = True
                    self.marks[self.round_out_bottom(nFrame.posX),self.round_out_bottom(nFrame.posY)] = 1
                    heapq.heappush(currPaths,nFrame)

            # down right
            for i in range(0,GRID_MAP_FRAME_SLICES):
                nFrame = self.child_frame(curr,entity,(aStep*i)+PI,destX,destY)
                if (self.in_bounds(nFrame.posX,nFrame.posY) and not (nFrame.get_id(self) in visited)):
                    visited[nFrame.get_id(self)] = True
                    self.marks[self.round_out_bottom(nFrame.posX),self.round_out_bottom(nFrame.posY)] = 1
                    heapq.heappush(currPaths,nFrame)

            # down left
            for i in range(1,GRID_MAP_FRAME_SLICES):
                nFrame = self.child_frame(curr,entity,(aStep*i*-1)+PI,destX,destY)
                if (self.in_bounds(nFrame.posX,nFrame.posY) and not (nFrame.get_id(self) in visited)):
                    visited[nFrame.get_id(self)] = True
                    self.marks[self.round_out_bottom(nFrame.posX),self.round_out_bottom(nFrame.posY)] = 1
                    heapq.heappush(currPaths,nFrame)

        print("no path exists :(")

    def __init__(self,unitSize,sizeX,sizeY):
        self.unitSize = unitSize
        self.sizeX = int(numpy.floor(sizeX/self.unitSize))
        self.sizeY = int(numpy.floor(sizeY/self.unitSize))
        self.entities = []
        self.grid = numpy.zeros((self.sizeX+1,self.sizeY+1))
        self.marks = numpy.zeros((self.sizeX+1,self.sizeY+1))


