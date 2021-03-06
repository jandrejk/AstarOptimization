import numpy as np


def load3DColorwash (path, setOneToInfty = False) :
    File = open(path,'r')
    lines = File.readlines()
    
    dim = list(map(int,lines[0].split()))
    gantry_angle_CC = list(map(float,lines[1].split()))
    table_angle_CC = list(map(float,lines[2].split()))


    colorlist = []
    for i in range(dim[1]):
        colorlist.append(list(map(float,lines[i+3].split())))

    colormap = np.array(colorlist).reshape([dim[1],dim[0]])
    if setOneToInfty :
        colormap[colormap > 0] = 3. #float("inf")
    
    return dim, gantry_angle_CC, table_angle_CC, colormap