#!/usr/bin/env python3

import sys
import numpy as np
import time
import astarVoxelClasses as AstarCl
from scipy.spatial import ConvexHull
sys.path.insert(0,'/home/smcp/janik/dto/scripts/utilities')
import util as ut
import pickle

def a_star_search(graph, ga_Pixel_range):
    
    start = (ga_Pixel_range[0],0)
    goal = ga_Pixel_range[1]
    
    frontier = AstarCl.PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
     
        
    while not frontier.empty():
        current = frontier.get()
        if current[0] == goal:
            break
        
        for next_point in graph.GetNeighbours(current) :
            new_cost = cost_so_far[current] + graph.cost(next_point)
            if next_point not in cost_so_far or new_cost < cost_so_far[next_point]:
                cost_so_far[next_point] = new_cost
                
                priority = new_cost + graph.heuristic(next_point)
                
                frontier.put(next_point, priority)
                came_from[next_point] = current
            
        
    return came_from, current


def GetParamsFromProperties (path) :
    File = open(path,'r')
    lines = File.readlines()
    
    gradient = int(lines[2][7:])
    
    
    see_future = int(lines[6][7:])
    
    mapTypeSelection = int(lines[9][10:])
    
    SAD = float(lines[15][7:])
    
    return gradient, see_future, mapTypeSelection, SAD 
    """
    debuging is missing
    """

def dot(a,b) :
    return np.linalg.multi_dot((a,b))


def projection (vector, BEV) :
    z_BEV = BEV[2]
    z_p = vector[2]
    
    lam = z_p / (z_BEV - z_p)
    
    
    return vector + lam * (vector - BEV)


"""
MAIN PROGRAM
"""    
print("============================")
go = time.time()
print("GC Astar algorithm started")

pathToProperties = sys.argv[1]
pathToPTVmesh = sys.argv[2]
pathToGTtrack = sys.argv[3]
pathToGCcolormap = sys.argv[4]
pathToOutputGTCFile = sys.argv[5]


#read properties file and get gantry range
grad, future, mapSelection, SAD = GetParamsFromProperties(path=pathToProperties)


# create GC search map

# get gantry-table track 
ga, ta = ut.LoadTrack(directory=pathToGTtrack)
# isocenter in hfs system 
iso = np.array([0.,0.,0.])


# get PTV vertices
PTV_vertices = ut.GetPTV_vertices(directory=pathToPTVmesh)


# BEV (beam's eye view) position 
BEV = np.array([0.,0.,SAD])


length = len(ga)
col_map = []

# min leaf distance option is selected
if mapSelection == 0 :
    #ONLY NEED TO DO HALF BECAUSE THEN PATTERN GETS REPEATED
    colli_angle = np.deg2rad(np.arange(-179.,180.,2.)) # [-175,175] in steps of 2
# area option is selected
else :
    colli_angle = np.deg2rad(np.arange(-179.,180.,2.))# [-179,179] in steps of 2
    colli_matrix2 = np.array([np.sin(-colli_angle),np.cos(-colli_angle)])

colli_matrix = np.array([np.cos(-colli_angle),-np.sin(-colli_angle)])
    

if mapSelection == 1 :
    print('generate dicts')
    jawX_dict = {}
    jawY_dict = {}



for i in range(length) :
    gantry = np.deg2rad(ga[i])
    table = np.deg2rad(ta[i])
    
    Rz = np.array([[np.cos(table), -np.sin(table), 0.],
               [np.sin(table),np.cos(table),0.],
               [0.,0.,1.]])

    Ry = np.array([[np.cos(gantry), 0 , -np.sin(gantry)],
             [0.,1.,0.],
             [np.sin(gantry),0.,np.cos(gantry)]])
    
    R = Ry.dot(Rz)
    # these points are rotated to be seen from beam's eze view direction
    PTV_vertices_trafo = np.array([dot(Ry,dot(Rz,v)) for v in PTV_vertices])
    PTV_vertices_trafo2 = np.array([R.dot(v) for v in PTV_vertices])
    
    for j in range(len(PTV_vertices_trafo)) :
        value = PTV_vertices_trafo[j] - PTV_vertices_trafo2[j]
        if abs(value).all() > 0.00000001 :
            print(j,'different')
    
    
    print(PTV_vertices_trafo.shape)
    
    # no perform conic projection from beam's eye view
    PTV_vertices_trafo_proj = np.array([projection(vector=v,BEV=BEV) for v in PTV_vertices_trafo])

    Q = PTV_vertices_trafo_proj
    
    # make a scan over all possible collimator rotations
    A = np.dot(Q[:,[0,1]],colli_matrix)
    jawX_dict[i] = (A.min(axis=0),A.max(axis=0))
    x_min_dist = A.max(axis=0) - A.min(axis=0)
    
    
    if mapSelection == 0 :
        col_map.append(x_min_dist)
    # area option chosen
    else :
        B = np.dot(Q[:,[0,1]],colli_matrix2)
        jawY_dict[i] = (B.min(axis=0),B.max(axis=0))
        y_min_dist = B.max(axis=0) - B.min(axis=0)
    
        col_map.append(np.multiply(x_min_dist,y_min_dist))

"""
if mapSelection == 0 :        
    GC_map = np.hstack((col_map,col_map)).T
else :
    GC_map = np.hstack((col_map,col_map,col_map,col_map)).T
"""

print(np.shape(col_map))
GC_map = np.array(col_map).T

print(np.shape(GC_map))

m = GC_map.min()
GC_map = (GC_map - m) / (GC_map.max()-m)

print('time until map is ready: %.3f sec'%(time.time()-go))

s = time.time()
# save the map as track-gap.txt 
ut.SaveMapToTxt(MAP=GC_map,directory=pathToGCcolormap)
print('save map needs %.3f sec'%(time.time()-s))



ss = time.time()
#convert gantry angles to pixels
#add (-1) to start search from the left and
#first point is already in the correct path
ga_start = -1#ut.GantryAngleToPixel(angle=ga[0])-1
ga_stop = length -1 #ut.GantryAngleToPixel(angle=ga[-1])

"""
print('gantry angles')
print(ga[0])
print(ga[-1])
print('----------')

print(ga_start)
print(ga_stop)
"""

GC = AstarCl.GC_map(color_map=GC_map,gradient=grad,seeTheFuture=future,
                    start=ga_start,end=ga_stop)



came_from, last_point = a_star_search(GC, ga_Pixel_range=[ga_start,ga_stop])

#get path and save it
path = AstarCl.reconstruct_path(came_from=came_from, start=(ga_start,0),goal=last_point)
print('search time %.3f sec'%(time.time()-ss))



ca = ut.PixelToColliAngle(pixel=np.array([i[1] for i in path]))

ut.SaveGTCToTxt(gantry_angle=ga, table_angle=ta, 
                colli_angle=ca, directory=pathToOutputGTCFile)


ut.SaveGTCJToTxt(gantry_angle=ga, table_angle=ta, 
                colli_angle=ca, gc_path = path, jawX = jawX_dict, 
                jawY=jawY_dict, directory='../tmp/gtcj-track.txt')


with open('../tmp/jawX.pkl','wb') as f : pickle.dump(jawX_dict,f)
with open('../tmp/jawY.pkl','wb') as f : pickle.dump(jawY_dict,f)

print(path)
print("GC Astar algorithm terminated")
print("time elapsed [s] ", time.time()-go)
print("============================")



