#!/usr/bin/env python3

import sys
import numpy as np
import time
import astarVoxelClasses as AstarCl
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
        #print('current',current)
        if current[0] == goal:
            break
        
        for next_point in graph.GetNeighbours(current):
            new_cost = cost_so_far[current] + graph.manhatten(current,next_point) * graph.cost(current, next_point) 
            if next_point not in cost_so_far or new_cost < cost_so_far[next_point]:
                cost_so_far[next_point] = new_cost
                
                priority = new_cost + graph.heuristic(next_point,cost_so_far[current]) #+ graph.manhatten(current,next_point)
                print('new_cost: %.3f heuristic: %.3f manhatten: %.3f'%(new_cost,graph.heuristic(next_point,cost_so_far[current]),graph.manhatten(current,next_point)))
                
                frontier.put(next_point, priority)
                came_from[next_point] = current
    
    return came_from, current


def GetParamsFromProperties (path) :
    File = open(path,'r')
    lines = File.readlines()
    
    gradient = int(lines[2][7:])
    
    range_values = lines[7].split(" ")
    
    lower_gantry_angle = float(range_values[0][7:])
    upper_gantry_angle = float(range_values[1][:-1])
    
    see_future = int(lines[11][7:])
    
    return lower_gantry_angle, upper_gantry_angle, gradient, see_future 
    """
    debuging is missing
    """
    
"""
MAIN PROGRAM
"""    
print("============================")
go = time.time()
print("Voxel aStar algorithm started")

pathToProperties = sys.argv[1]
pathToOARs = sys.argv[2]
pathToColl = sys.argv[3]
pathToCTrestr = sys.argv[4]
pathToOutputFile = sys.argv[5]

#read properties file and get gantry range
lower_ga, upper_ga, grad, future = GetParamsFromProperties(path=pathToProperties)

#convert gantry angles to pixels
#add (-1) to start search from the left and
#first point is already in the correct path
ga_start = ut.GantryAngleToPixel(angle=lower_ga)-1
ga_stop = ut.GantryAngleToPixel(angle=upper_ga)


#load maps
collMap = ut.load3DColorwash(pathToColl,setOneToInfty=False)[3]
cTrestrMap = ut.load3DColorwash(pathToCTrestr,setOneToInfty=False)[3]

dim, ga, ta, oARmap = ut.load3DColorwash2(pathToOARs)

GT = AstarCl.GT_map(oARmap,collMap,cTrestrMap,gradient=grad,seeTheFuture=future,
                    start=ga_start,end=ga_stop)


came_from, last_point = a_star_search(GT, ga_Pixel_range=[ga_start,ga_stop])

#get path and save it
path = AstarCl.reconstruct_path(came_from=came_from, start=(ga_start,0),goal=last_point)


#print(last_point)
#print(ga_start)
#print(ga_stop)

path_gantry_angle = ut.PixelToGantryAngle(pixel=np.array([i[0] for i in path]))
path_table_angle = ut.PixelToTableAngle(pixel=np.array([i[1] for i in path]))

ut.SaveAnglesToTxt(gantry_angle=path_gantry_angle, 
                   table_angle=path_table_angle, directory=pathToOutputFile) 

print("Voxel aStar algorithm terminated")
print("time elapsed [s] ", time.time()-go)
print("============================")




