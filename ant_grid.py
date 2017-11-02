import random
import ant as ANT
import numpy as np

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
                    self.tau_ij[t,ant.track[i+1]] = 25.
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

                for i,t in enumerate(ant.track[:-1]) :
                    pheromone_value = self.tau_ij[t,ant.track[i+1]]

                    #if pheromone_value < 20. :
                    if (pheromone_value < 20. and pheromone_value > 0.5 ) :
                        self.tau_ij[t,ant.track[i+1]] = pheromone_value + cost_value / (1.-rho) #* (1.-rho) / no_of_ants 
                        if ant.critical_steps > 1 :
                            x_nest = ant.track[0][0]
                            x_died = ant.track[-1][0]
                            diff = x_died - x_nest

                            reduction = np.minimum(0.9,(diff/10.)**2 / 10.)

                            #print reduction# (np.log(x_died-x_nest))
                            self.tau_ij[t,ant.track[i+1]] = pheromone_value*reduction#0.2*(np.log(x_died-x_nest)) 


                cost_in_generation.append(ant.cost)

            """    
            maxi = 0.    
            for key in self.tau_ij:    
                self.tau_ij[key] *=  (1.-rho)
                if self.tau_ij[key] > maxi :
                    maxi = self.tau_ij[key]
            print 'maximal pheromone is      
            """    
        #self.DeamonFeromone(ant=colony[np.argmin(cost_in_generation)],rho=rho)

    def DeamonFeromone (self,ant,rho,boost_factor=5.) :
        track_length = len(ant.track)*1.
        track_cost_perpixel = ant.cost / track_length / boost_factor
        
        cost_value = 1. / track_cost_perpixel
        
        for i,t in enumerate(ant.track[:-1]) :
            self.tau_ij[t,ant.track[i+1]] = self.tau_ij[t,ant.track[i+1]] * (1.-rho) + cost_value


            
            
def GetNextState (state_transition_probas,neigbours) :
    
    #perform random shuffeling of the 2 lists:
    #package = list(zip(state_transition_probas,neigbours))
    #random.shuffle(package)
    #probas, n = zip(*package)
    
    probas = state_transition_probas
    n = neigbours
    
    #print probas
    #print n
    
    random_uniform = np.random.rand()
    #print random_uniform
    summing = 0
    for i, p in enumerate(probas) :
        summing += p
        #print summing
        #print summing>=p
        if summing >= random_uniform :
            next_state = n[i]
            return next_state            