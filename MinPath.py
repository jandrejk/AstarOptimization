#!/usr/bin/env python3

import sys
import numpy as np
sys.path.insert(0,'/home/smcp/janik/dto/scripts/utilities')
import util as ut

def MinPath (OaR,coll,CT,output_file,min_ga,max_ga) :
    pathToOARs    = OaR
    pathToColl    = coll
    pathToCTrestr = CT
    
    lower_bound = ut.GantryAngleToPixel(angle=min_ga)
    upper_bound = ut.GantryAngleToPixel(angle=max_ga)
    
   
    #load the maps
    collMap = ut.load3DColorwash(pathToColl,setOneToInfty=True)[3]
    cTrestrMap = ut.load3DColorwash(pathToCTrestr,setOneToInfty=True)[3]
    dim, ga, ta, oARmap = ut.load3DColorwash(pathToOARs)
    
    #generate one single map, i.e. combine OaR, coll and CT
    forbidden = np.maximum(cTrestrMap,collMap) 
    gt_map = np.maximum(oARmap,forbidden) 
    
    table = np.argmin(gt_map,axis=0)[lower_bound:upper_bound]
    table_angle = ut.PixelToTableAngle(pixel=table)
    
    #save gt track to txt file
    ut.SaveAnglesToTxt(gantry_angle=ga[lower_bound:upper_bound],
                       table_angle=table_angle,directory=output_file)


def GetParamsFromProperties (path) :
    File = open(path,'r')
    lines = File.readlines()
    range_values = lines[3].split(" ")
    
    lower_gantry_angle = float(range_values[0][7:])+0.5
    upper_gantry_angle = float(range_values[1][:-1])-0.5
    
    
    print(lower_gantry_angle)
    print(upper_gantry_angle)
    
    return lower_gantry_angle, upper_gantry_angle
    

"""
MAIN PROGRAM
"""    
print("============================")
print("minpath algorithm started")

pathToProperties = sys.argv[1]
pathToOARs = sys.argv[2]
pathToColl = sys.argv[3]
pathToCTrestr = sys.argv[4]
pathToOutputFile = sys.argv[5]

#read properties file and get gantry range
lower_ga, upper_ga = GetParamsFromProperties(path=pathToProperties)


MinPath(OaR=pathToOARs,coll=pathToColl,CT=pathToCTrestr,output_file=pathToOutputFile,
        min_ga=lower_ga, max_ga=upper_ga)

print("minpath algorithm terminated")
print("============================")




