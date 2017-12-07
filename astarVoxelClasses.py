import numpy as np
import heapq

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    
class GT_map :
    def __init__(self, color_map,collision_map, CT_map, gradient=1,start=0,end=180,seeTheFuture=1) :
        self.color = color_map
        self.collision = collision_map
        self.ct = CT_map
        self.grad = gradient
        self.start_ga_Pixel = start
        self.end_ga_Pixel = end
        self.future = seeTheFuture
        
        self.tableMaxPixel, self.gantryMaxPixel = np.shape(self.color)
        
        
    def GetNeighbours (self, point) :
        """
        params:
            point      - is of type tuple. represents the coordinates (x,y) in pixel
                         units, i.e. in terms of the dimension of the color map
        returns :
            neighbours - is a list of tuples. Each list element is a tuple (x,y) of
                         points being the nrighbours of the input point
        """

        x = point[0]
        y = point[1]
        
        
        if (x == self.start_ga_Pixel) :
            neighbours = []
            for t in range(self.tableMaxPixel) :
                neighbours.append((x+1,t))
        else :
            #go to right
            neighbours = [(x+1,y)]
            for g in range(self.grad) :
                neighbours.append((x+1,y+(1+g)))
                neighbours.append((x+1,y-(1+g)))

        forbidden = []
        for n in neighbours :
            if (n[0]<0 or n[0]>self.gantryMaxPixel-1 or n[1]<0 or n[1]>self.tableMaxPixel-1
               or self.collision[n[::-1]] != 0. or self.ct[n[::-1]] != 0.) :
                forbidden.append(n)
            
        for f in forbidden :
            neighbours.remove(f)

        return neighbours
    
    def cost (self, current_point, next_point) :
        return self.color[next_point[::-1]]
    
    def heuristic(self,next_point,mult_value) :
        x = next_point[0]
        y = next_point[1]
        
        #return (np.sum(self.color[y][x:x+self.future]) / self.future + 1. + np.log(self.future)) * mult_value
        return (np.sum(self.color[y][x:x+self.future]) )  # / self.future + 1. + np.log(self.future)) * mult_value
    
    def manhatten(self, current, next_point) :
        return (abs(next_point[0] - current[0]) + abs(next_point[1]-current[1])) 
    
    def heuristic2(self,next_point,mult_value) :
        x = next_point[0]
        y = next_point[1]
        window = self.future
        #return (np.sum(self.color[y][x:x+self.future]) / self.future + 1. + np.log(self.future)) * mult_value
        return (np.sum(self.color[y][x-window:x+window]) )  # / self.future + 1. + np.log(self.future)) * mult_value
    
def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    
    #append start point being one to the left
    #do not append because go to the left at the beginning
    #path.append((path[-1][0]-1,path[-1][1])) 
    
    path.reverse() 
    return path 





class GC_map :
    def __init__(self, color_map, gradient=1,start=0,end=180,seeTheFuture=3) :
        self.color = color_map
        self.grad = gradient
        self.start_ga_Pixel = start
        self.end_ga_Pixel = end
        self.future = seeTheFuture
        
        self.colliMaxPixel, self.gantryMaxPixel = np.shape(self.color)
        
        
    def GetNeighbours (self, point) :
        """
        params:
            point      - is of type tuple. represents the coordinates (x,y) in pixel
                         units, i.e. in terms of the dimension of the color map
        returns :
            neighbours - is a list of tuples. Each list element is a tuple (x,y) of
                         points being the nrighbours of the input point
        """

        x,y = point
        
        neighbours = [(x+1,t) for t in range(self.colliMaxPixel)]
        
        #re-map function GetNeighbours at runtime
        self.GetNeighbours = self.GetNeighbours_post
        
        #first time there is no forbidden pixel outside the map
        
        return neighbours

    def GetNeighbours_post (self, point) :
        """
        params:
            point      - is of type tuple. represents the coordinates (x,y) in pixel
                         units, i.e. in terms of the dimension of the color map
        returns :
            neighbours - is a list of tuples. Each list element is a tuple (x,y) of
                         points being the nrighbours of the input point
        """

        x,y = point      
       
        #go to right
        neighbours = [(x+1,y)]
        for g in range(self.grad) :
            neighbours.append((x+1,y+(1+g)))
            neighbours.append((x+1,y-(1+g)))

        
        forbidden = []
        for n in neighbours :
            if (n[1]<0 or n[1]>self.colliMaxPixel-1) :
                forbidden.append(n)
            
        for f in forbidden :
            neighbours.remove(f)

        """    
        neighbours[:] = [n for n in neighbours if (n[0]>=0 and n[0]<=self.gantryMaxPixel-1 and n[1]>=0 and n[1]<=self.tableMaxPixel-1
               and self.collision[n[::-1]] == 0. and self.ct[n[::-1]] == 0.)]
        """
        
        return neighbours
    
    def cost (self, next_point) :
        return self.color[next_point[::-1]]
    
    def heuristic(self,next_point) :
        x,y = next_point
        return np.sum(self.color[y][x:x+self.future])

    