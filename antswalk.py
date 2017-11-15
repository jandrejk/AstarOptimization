#!/usr/bin/env python3

import sys
import numpy as np
import time
import ant as ANT
import ant_grid as GRID
from itertools import repeat
from multiprocessing import Pool


sys.path.insert(0,'/home/smcp/janik/dto/scripts/utilities')
import util as ut





def GetMaxProbaPath (start, goal, decisionTable) :
    path = []
    path.append(start) 
    
    current = start
    while current[0] != goal[0] :
        max_proba = 0.
        for n in ANT.GetNeighbours(point=current) :
            if n not in path :
                proba = decisionTable.p_ij(location=current,destination=n,track=path)
                #print(proba)
                if proba > max_proba :
                    max_proba = proba
                    next_point = n
        #print (next_point)
        path.append(next_point)
        current = next_point
    
    return path



def ant_walk (ant,food) :
    while (ant.location != food) :

        loc = ant.location
        state_transition_probas = []

        neigbours = ANT.GetNeighbours(point=loc)


        for n in neigbours :
            state_transition_probas.append(DecTable.p_ij(location=loc,destination=n,track=ant.track))

        next_state = GRID.GetNextState(state_transition_probas=state_transition_probas,neigbours=neigbours)
        ant.AddMove(nextpoint=next_state)
        
        print(next_state)
        colormap_value = 1./ DecTable.eta_ij[loc,next_state]
        ant.AddCost(value=colormap_value)

        #print colormap_value
        if colormap_value >= cost_goal_generation :
            ant.critical_steps += 1

        if ant.critical_steps > 1 :
            #print 'dead'
            no_of_critical_ants += 1
            break



        # if level reached of the food one, go up or down
        if (ant.location[0] == food[0]) :

            while loc[1] != food[1] :
                loc = ant.location

                #walk up
                if loc[1] < food[1] :
                    next_state = (loc[0],loc[1]+1)
                else :
                    next_state = (loc[0],loc[1]-1)

                ant.AddMove(nextpoint=next_state)
                #print 1./DecTable.eta_ij[loc,next_state]
                colormap_value = 1./ DecTable.eta_ij[loc,next_state]
                ant.AddCost(value=colormap_value)

                #print colormap_value

                if colormap_value >= cost_goal_generation :
                    ant.critical_steps += 1

                if ant.critical_steps > 1 :
                    #print 'dead'
                    no_of_critical_ants += 1
                    break

            break





def progressBar(value, endvalue, bar_length=20):

        percent = float(value) / endvalue
        arrow = '-' * int(round(percent * bar_length)-1) + '>'
        spaces = ' ' * (bar_length - len(arrow))

        sys.stdout.write("\rDone: [{0}] {1}%".format(arrow + spaces, int(round(percent * 100))))
        sys.stdout.flush()            
            
            
def AOC_path (nest, food, DecTable, no_of_colonies,no_of_ants=5) :
    """
    This function return the best (maximum proability) ant path connecting the nest with food.
    """
    #go = time.time()
    no_of_critical_ants = 0
    cost_goal_generation = 0.3
    
    
    for generation in range(no_of_colonies) :
        #print('colony',no_of_colonies)
        #progressBar(value = generation, endvalue=no_of_colonies)
        #sys.stdout.write("done: " % generation / no_of_colonies * 100.)
        #sys.stdout.flush()
        if no_of_critical_ants < 30 :
            cost_goal_generation -= 0.001 
        else : cost_goal_generation += 0.001

 
        ant_colony = []
        no_of_critical_ants = 0
        
        
        for m in range(no_of_ants) :
            #print('ant',m)
            ant = ANT.Ant(birth_coordinates=nest)
           
            ant_colony.append(ant) 

            while (ant.location != food) :

                loc = ant.location
                state_transition_probas = []

                neigbours = ANT.GetNeighbours(point=loc)


                for n in neigbours :
                    state_transition_probas.append(DecTable.p_ij(location=loc,destination=n,track=ant.track))
                
                next_state = GRID.GetNextState(state_transition_probas=state_transition_probas,neigbours=neigbours)
                ant.AddMove(nextpoint=next_state)
                #print(loc,next_state)
                
                colormap_value = 1./ DecTable.eta_ij[loc,next_state]
                ant.AddCost(value=colormap_value)

                if colormap_value >= cost_goal_generation :
                    ant.critical_steps += 1

                if ant.critical_steps > 1 :
                    #print 'dead'
                    no_of_critical_ants += 1
                    break



                # if level reached of the food one, go up or down
                if (ant.location[0] == food[0]) :
                    #print('level reached')
                    #print('food',food)
                    while ( loc[1] != food[1] and loc[1] > 1 and loc[1] < 88) :
                        loc = ant.location

                        #walk up
                        if loc[1] < food[1] :
                            next_state = (loc[0],loc[1]+1)
                        else :
                            next_state = (loc[0],loc[1]-1)
                        
                        #print(next_state)
                        ant.AddMove(nextpoint=next_state)
                        #print 1./DecTable.eta_ij[loc,next_state]
                        colormap_value = 1./ DecTable.eta_ij[loc,next_state]
                        ant.AddCost(value=colormap_value)

                        #print colormap_value

                        if colormap_value >= cost_goal_generation :
                            ant.critical_steps += 1

                        if ant.critical_steps > 1 :
                            #print 'dead'
                            no_of_critical_ants += 1
                            break
                    else :
                        break
                    break


        #print('ok')
        DecTable.UpdatePheromone(colony=ant_colony)
        #print('ok')

        """
        for n in ANT.GetNeighbours((108,45)) :
            print n
            print DecTable.p_ij(location=(108,45), destination=n)
        """

        #print ('reduced goal', cost_goal_generation)
        #print( 'no_of_critical_ants', no_of_critical_ants)

        """
        if generation%100 == 0 :
            print (generation)

            plt.figure(figsize=(11,11))
            pl.FlooadPlot(pic=gt_map,flood_level=0.3)

            cost_in_generation = []
            for a in ant_colony :
                track_length = 1.*len(a.track)
                pl.PlotTrack(track=a.track)
                #plt.pause(0.05)
                if a.critical_steps < 2 :
                    #print 'hi'
                    cost_in_generation.append(a.cost / track_length)
                else :   
                    cost_in_generation.append(1.)
                #print TrackCost(ant=a,cost_map=astar_landscape)
            print ('minimal track', np.argmin(cost_in_generation))
            print ('cost', ant_colony[np.argmin(cost_in_generation)].cost)
            print ('cost per unit length', ant_colony[np.argmin(cost_in_generation)].cost / len(ant_colony[np.argmin(cost_in_generation)].track))
            print ('amount of critical steps', ant_colony[np.argmin(cost_in_generation)].critical_steps)


            pl.HighlightTrack(ant_colony[np.argmin(cost_in_generation)].track)
            plt.show()
        """
    stop = time.time()

    #print ('total time: ', stop-go)
    return GetMaxProbaPath(start=nest,goal=food,decisionTable=DecTable)
    

def GetParamsFromProperties (path) :
    File = open(path,'r')
    lines = File.read()
    info = lines.split('\n')[3::5]
    
    beta = float(info[0][7:])
    thres = float(info[1][7:])
    no_iterations = int(info[2][7:])
    no_ants = int(info[3][7:])
    alpha = float(info[4][7:])
    
    return no_iterations, no_ants, alpha, beta, thres


def NestFoodCombinations (gt_map, threshold) :
    """
    return longest path of connected minima
    """
    table_min = np.argmin(gt_map,axis=0)
    print(len(table_min))
    connectivity = []
    sub_path = []
    for gantry, table in enumerate(table_min) :
        #print(gt_map[(gantry,table)[::-1]])
        if gt_map[(gantry,table)[::-1]] < threshold :
            sub_path.append(gantry)
        else :
            connectivity.append(sub_path)
            sub_path = []
    connectivity.append(sub_path)
    
    lengths = [len(l) for l in connectivity]
    
    longest_min_path = connectivity[np.argmax(lengths)]
    start_path = longest_min_path[0]
    end_path = longest_min_path[-1]
    print(start_path,end_path)
    #compute nest food combinations
    y = []
    end = end_path
    stick_length = int((end_path-start_path)/8)


    x = start_path
    while x < end :   
        y.append(np.argmin(np.sum(gt_map[:,x:x+stick_length],axis=1)))
        x += stick_length
    """
    if x < end_path :
        y.append(np.argmin(gt_map[:,end_path]))
    """
    x = start_path
    i = 0

    nest_food_combinations = []

    while x+stick_length < end :   
        nest_food_combinations.append((x,y[i]))
        x += stick_length
        i += 1

    run = True
    while run :
        run = False
        for i,point in enumerate(nest_food_combinations[:-1]) :
            next_point = nest_food_combinations[i+1]
            if abs(point[1]-next_point[1]) > 60 :
                
                new_x = int((next_point[0] + point[0]) / 2)
                new_y = int((next_point[1] + point[1]) / 2)
                
                nest_food_combinations[i] = (new_x,new_y)
                #nest_food_combinations[i+1] = (new_x,new_y)
                #print(point,next_point)
                #print('too large gradient')
                run = True
                
        #print(run,'run output')
    return nest_food_combinations


def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    
    for i in range(box_pts) :
        y_smooth[i] = y[i]
        y_smooth[-i] = y[-i]
        
    
    #for i,fac in enumerate(np.arange(box_pts,2*box_pts,1)) :
    #    y_smooth[i]  = y_smooth[i] * 2 * box_pts / fac
    #    y_smooth[-i] = y_smooth[-i] * 2 * box_pts / fac
    #print(y_smooth)
    return y_smooth


"""
MAIN PROGRAM
"""    
print("============================")
print("ant algorithm started")

pathToProperties = sys.argv[1]
pathToOARs = sys.argv[2]
pathToColl = sys.argv[3]
pathToCTrestr = sys.argv[4]
pathToOutputFile = sys.argv[5]

#read properties file and get gantry range
no_iterations, no_ants, alpha, beta, thres = GetParamsFromProperties(path=pathToProperties)

#get maps
collMap = ut.load3DColorwash(pathToColl,setOneToInfty=True)[3]
cTrestrMap = ut.load3DColorwash(pathToCTrestr,setOneToInfty=True)[3]
dim, ga, ta, oARmap = ut.load3DColorwash(pathToOARs)
#define ant gt map as combination of loaded maps
forbidden = np.maximum(cTrestrMap,collMap) 
gt_map = np.maximum(oARmap,forbidden) 


#Pre-Processing
nest_food_combinations = NestFoodCombinations(gt_map=gt_map, threshold=thres)
print(nest_food_combinations)
#run algorithm

pheromone_0 = {}
set_pheromone_const =  10.

dist_cost = {}

for x in range(180) :
    for y in range(90) :
        for n in ANT.GetNeighbours((x,y)) :
            pheromone_0[(x,y),n] = set_pheromone_const
            dist_cost[(x,y),n] = 1. / gt_map[n[::-1]]


DecTable = GRID.AntDecisionTable(pheromone=pheromone_0, distance=dist_cost,alpha=alpha,beta=beta)

go = time.time()
full_path = []
with Pool(20) as p :
    full_path = p.starmap(AOC_path,zip(nest_food_combinations[:-1],nest_food_combinations[1:],repeat(DecTable),repeat(no_iterations),repeat(no_ants)))

print('time for full path', time.time()-go)

nice_x = [i[0] for i in np.vstack(full_path)]
y = [i[1] for i in np.vstack(full_path)]
nice_y = smooth(y,4)


ut.SaveAnglesToTxt(gantry_angle=ut.PixelToGantryAngle(pixel=np.array(nice_x)), table_angle=ut.PixelToTableAngle(pixel=np.array(nice_y)),directory=pathToOutputFile)
    


print("ant algorithm terminated")
print("============================")
