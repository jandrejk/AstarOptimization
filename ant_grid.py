import random
import ant as ANT
import numpy as np
import time

class AntDecisionTable :
    
    def __init__(self,pheromone,distance,alpha=1.,beta=5.) :

        self.eta_ij = distance 
        self.tau_ij = pheromone
        self.alpha = alpha
        self.beta = beta
    
    def a_ij (self, location, destination) :
        loc = location
        dest = destination
        numerator = (self.tau_ij[loc,dest])**self.alpha * self.eta_ij[loc,dest]**self.beta
        
        denominator = 0.
        for n in ANT.GetNeighbours(point=loc) :
            denominator += (self.tau_ij[loc,n])**self.alpha * self.eta_ij[loc,n]**self.beta
        
        a_ij = numerator / denominator
        return a_ij
    
    def p_ij (self, location, destination, track) :
        
        if destination not in track :
            loc = location
            dest = destination
            numerator = self.a_ij(location=loc,destination=dest)
            denominator = 0

            for n in ANT.GetNeighbours(point=loc) :
                if n not in track :
                    denominator += self.a_ij(location=loc,destination=n)
        
            p_ij = numerator / denominator
            return p_ij
        else :
            return 0.
    def UpdatePheromone(self,colony,rho=0.5) :
        cost_in_generation = []
        no_of_ants = 1.*len(colony)
        for ant in colony :
            if ant.critical_steps <= 1 :
                for i,t in enumerate(ant.track[:-1]) :
                    #self.tau_ij[t,ant.track[i+1]] = 25.
                    pheromone_value = self.tau_ij[t,ant.track[i+1]]
                    track_length = len(ant.track)*1.
                    cost_value = 1. / (ant.cost / track_length * np.log(track_length)) 
                
                    if (pheromone_value < 30. and pheromone_value > 0.5 ) :
                        self.tau_ij[t,ant.track[i+1]] = pheromone_value + cost_value / (1.-rho) #* (1.-rho) / no_of_ants 
                    
            else :   
                #if ant.critical_steps < 5 :
                #cost_value = 1. / ant.cost
                track_length = len(ant.track)*1.
                #cost_value = track_length * cost_value

                """
                print 'ant no. ', i
                print 'cost per pixel step', ant.cost / track_length
                print 'antcost', ant.cost
                print 'resulting update',  1. / (ant.cost / track_length )
                """

                #cost_value = 1. / (ant.cost / track_length ) 
                cost_value = 1. / (ant.cost / track_length * np.log(track_length)) 
                #print 'cost of ant',cost_value
                
                reduction_smoothness = 0.033
                for i,t in enumerate(ant.track[:-1]) :
                    pheromone_value = self.tau_ij[t,ant.track[i+1]]

                    #if pheromone_value < 20. :
                    if (pheromone_value < 30. and pheromone_value > 0.5 ) :
                        #self.tau_ij[t,ant.track[i+1]] = pheromone_value + cost_value / (1.-rho) #* (1.-rho) / no_of_ants 
                        
                        if ant.critical_steps > 1 :
                            x_nest = ant.track[0][0]
                            x_died = ant.track[-1][0]
                            diff = x_died - x_nest

                            reduction = np.minimum(0.9,(diff/10.)**2 / 10.)
                            
                            
                            #print reduction# (np.log(x_died-x_nest))
                            self.tau_ij[t,ant.track[i+1]] = pheromone_value*(0.999 - i * (0.999*reduction) / (track_length-1.))#0.2*(np.log(x_died-x_nest)) 
                        else :
                            print('you are wrong')
                    #else:                    #     print 'phero value', pheromone_value
                cost_in_generation.append(ant.cost)

        #print 'new ant'      
            #maxi = 0.    
            #for key in self.tau_ij:    
            #    self.tau_ij[key] *=  1. #(1.-rho)
                #if self.tau_ij[key] > maxi :
                #    maxi = self.tau_ij[key]
            #print 'maximal pheromone is      
                
        #self.DeamonFeromone(ant=colony[np.argmin(cost_in_generation)],rho=rho)

    def DeamonFeromone (self,ant,rho,boost_factor=5.) :
        track_length = len(ant.track)*1.
        track_cost_perpixel = ant.cost / track_length / boost_factor
        
        cost_value = 1. / track_cost_perpixel
        
        for i,t in enumerate(ant.track[:-1]) :
            self.tau_ij[t,ant.track[i+1]] = self.tau_ij[t,ant.track[i+1]] * (1.-rho) + cost_value


            
            
def GetNextState (state_transition_probas,neigbours) :
    
    probas = state_transition_probas
    n = neigbours
        
    random_uniform = np.random.rand()

    summing = 0.
    for i, p in enumerate(probas) :
        summing += p
        if summing >= random_uniform :
            next_state = n[i]
            return next_state   
    

    
    
    
class AOC_path :
    """
    This function return the best (maximum proability) ant path connecting the nest with food.
    """
    
    def __init__(self, nest, food, gt_map, params, no_of_colonies = 30, no_of_ants=30, cost_goal_generation = 0.3) :
        self.nest = nest
        self.food = food 
        self.landscape = gt_map
    
    
    go = time.time()
    cost_goal_generation = 0.3
    
    """
    # loop over the number of generations
    for generation in xrange(no_of_colonies) :
        
        ant_colony = []
        no_of_critical_ants = 0
        
        
        for m in xrange(no_of_ants) :
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



        DecTable.UpdatePheromone(colony=ant_colony)

      
        print('reduced goal', cost_goal_generation)
        print('no_of_critical_ants', no_of_critical_ants)


        if generation%100 == 0 :
            print(generation)

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
            print('minimal track', np.argmin(cost_in_generation))
            print('cost', ant_colony[np.argmin(cost_in_generation)].cost)
            print('cost per unit length', ant_colony[np.argmin(cost_in_generation)].cost / len(ant_colony[np.argmin(cost_in_generation)].track))
            print('amount of critical steps', ant_colony[np.argmin(cost_in_generation)].critical_steps)


            pl.HighlightTrack(ant_colony[np.argmin(cost_in_generation)].track)
            plt.show()

    
    
        # decrease cost target for the next generation
        # if some ants reached the goal
        if no_of_critical_ants < no_of_ants :
            cost_goal_generation -= 0.001 
        else : 
            cost_goal_generation += 0.001

    stop = time.time()

    print('total time: ', stop-go)
    return GetMaxProbaPath(start=nest,goal=food,decisionTable=DecTable) 
    """