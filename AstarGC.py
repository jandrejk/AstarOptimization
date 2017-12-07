#!/usr/bin/env python3

import sys
import numpy as np
import time
import astarVoxelClasses as AstarCl
from scipy.spatial import ConvexHull
sys.path.insert(0,'/home/smcp/janik/dto/scripts/utilities')
import util as ut


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
# BEV (beam's eye view) position matrix
BEV = np.zeros(np.shape(PTV_vertices)) 
BEV[:,2] += SAD

length = len(ga)
col_map = []

# min leaf distance option is selected
if mapSelection == 0 :
    #ONLY NEED TO DO HALF BECAUSE THEN PATTERN GETS REPEATED
    colli_angle = np.deg2rad(np.arange(-179.,0.,2.)) # [-179,179] in steps of 2
# area option is selected
else :
    colli_angle = np.deg2rad(np.arange(-179.,-90.,2.)) # [-179,179] in steps of 2
    colli_matrix2 = np.array([np.sin(colli_angle),np.cos(colli_angle)])

colli_matrix = np.array([np.cos(colli_angle),-np.sin(colli_angle)])
    


for i in range(length) :
    gantry = np.deg2rad(ga[i])
    table = np.deg2rad(ta[i])
    
    # reduced rotation matrices because have 2d problem after projection
    # have rotation around z-axis with neg table angle and roation
    # around y-axis with negative gantry angle
    Rz_red = np.array([[np.cos(-table), -np.sin(-table)],
               [np.sin(-table),np.cos(-table)]])

    Ry_red = np.array([[np.cos(-gantry), -np.sin(-gantry)],
                         [np.sin(-gantry),np.cos(-gantry)]])

    M1 = np.dot(Rz_red,PTV_vertices[:,[0,1]].T) 
    y_trafo = M1[1,:]
    
    M2 = np.dot(Ry_red,np.vstack((M1[0,:],PTV_vertices[:,2])) )
    x_trafo = M2[0,:]
    z_trafo = M2[1,:]
    
    # these points are rotated to be seen from beam's eze view direction
    PTV_vertices_trafo = np.vstack((x_trafo,y_trafo,z_trafo)).T
    
    # no perform conic projection from beam's eye view
    z_points = PTV_vertices_trafo[:,2]
    lambda_vec = np.divide(z_points,BEV[:,2]-z_points)
    PTV_vertices_trafo_proj = PTV_vertices_trafo + np.multiply((PTV_vertices_trafo-BEV).T,lambda_vec).T
    
    # trick: use only the convex hull to speed up the algorithm because inner
    # points will do not have to be tested for min and max
    points = (PTV_vertices_trafo_proj[:,[0,1]])  
    hull = ConvexHull(points)
    Q = PTV_vertices_trafo_proj[np.hstack((hull.simplices[:,0],hull.simplices[:,1]))]
    
    # make a scan over all possible collimator rotations
    A = np.dot(Q[:,[0,1]],colli_matrix)
    x_min_dist = np.max(A,axis=0) - A.min(axis=0)
    
    if mapSelection == 0 :
        col_map.append(x_min_dist)
    # area option chosen
    else :
        B = np.dot(Q[:,[0,1]],colli_matrix2)
        y_min_dist = B.max(axis=0) - B.min(axis=0)
    
        col_map.append(np.multiply(x_min_dist,y_min_dist))

if mapSelection == 0 :        
    GC_map = np.hstack((col_map,col_map)).T
else :
    GC_map = np.hstack((col_map,col_map,col_map,col_map)).T

GC_map /= GC_map.max()

# save the map as track-gap.txt 
ut.SaveMapToTxt(MAP=GC_map,directory=pathToGCcolormap)




#convert gantry angles to pixels
#add (-1) to start search from the left and
#first point is already in the correct path
ga_start = ut.GantryAngleToPixel(angle=ga[0])-1
ga_stop = ut.GantryAngleToPixel(angle=ga[-1])


GC = AstarCl.GC_map(color_map=GC_map,gradient=grad,seeTheFuture=future,
                    start=ga_start,end=ga_stop)



came_from, last_point = a_star_search(GC, ga_Pixel_range=[ga_start,ga_stop])

#get path and save it
path = AstarCl.reconstruct_path(came_from=came_from, start=(ga_start,0),goal=last_point)



ca = ut.PixelToColliAngle(pixel=np.array([i[1] for i in path]))

ut.SaveGTCToTxt(gantry_angle=ga, table_angle=ta, 
                colli_angle=ca, directory=pathToOutputGTCFile)

print("GC Astar algorithm terminated")
print("time elapsed [s] ", time.time()-go)
print("============================")



