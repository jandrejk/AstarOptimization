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


def PixelToTableAngle (pixel) :
    return pixel*2. - 90.


def GantryAngleToPixel (angle) :
    return int((angle+180) / 2)

def SaveAnglesToTxt (gantry_angle, table_angle, directory) :
    file = open(directory,'w')
    
    file.write(str(len(gantry_angle)) + '\n')

    for i in range(len(gantry_angle)) :
        file.write(str(gantry_angle[i]) + '\t' + str(table_angle[i])+ '\n')

    file.close()