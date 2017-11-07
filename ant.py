class Ant :
    
    
    def __init__(self,birth_coordinates) :
        self.location = birth_coordinates
        self.track = [birth_coordinates]
        self.cost = 0.
        self.critical_steps = 0
        
    def AddMove (self,nextpoint) :
        self.location = nextpoint
        self.track.append(nextpoint)
    
    def AddCost (self,value) :
        self.cost += value

        
def GetNeighbours (point) :
    """
    This function returns the points/pixels being the neighbours of the input point.
    In this set-up the neighbours can be either above, below or to the right of the 
    input point illustrated below:   
    
    n n
    i n
    n n
    
    i stands for input point and n denotes the neighbours.
    
    params:
        point      - is of type tuple. represents the coordinates (x,y) in pixel
                     units, i.e. in terms of the dimension of the color map
    returns :
        neighbours - is a list of tuples. Each list element is a tuple (x,y) of
                     points being the nrighbours of the input point
    """
    x = point[0]
    y = point[1]
    
    #neighbours = [(x+1,y+1),(x+1,y),(x+1,y-1)]
    neighbours = [(x,y+1),(x+1,y+1),(x+1,y),(x+1,y-1),(x,y-1)]
    #neighbours = [(x,y+1),(x+1,y+1),(x+1,y),(x+1,y-1),(x,y-1),(x-1,y-1),(x-1,y),(x-1,y+1)]

    forbidden = []
    for n in neighbours :
        if (n[0]<0 or n[0]>179 or n[1]<0 or n[1]>89) :
            forbidden.append(n)
    
    for f in forbidden :
        neighbours.remove(f)
    
    return neighbours
    
            