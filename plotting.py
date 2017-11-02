import numpy as np
import matplotlib.pyplot as plt


def QuickPlot (pic) :
    plt.figure(figsize=(11,11))
    plt.imshow(pic,origin='left',cmap='jet')
    plt.colorbar(orientation='horizontal')
    
def FlooadPlot (pic, flood_level) :
    plt.figure(figsize=(11,11))
    flood = np.minimum(pic,np.zeros(np.shape(pic))+flood_level)
    plt.imshow(flood,origin='left',cmap='jet')
    plt.colorbar(orientation='horizontal')
    #plt.show()
    
    
def PlotTrack (track) :
    plt.plot(*zip(*track),color='k',marker='D',linestyle='')
    
def HighlightTrack (track) :
    plt.plot(*zip(*track),color='m',linestyle='-',lw=2)
    
def LinePlot (pic, start=0) :
  
    height, width = np.shape(pic)
       
            
    while start < width :
        
        arr = pic[:,start]
        q = np.argmin(pic[:,start])
        
            
        if pic[q,start] < 0.25 :
            plt.plot(start,q,'mo')
        else :
            plt.plot(start,q,'ko')
        
        start += 30
    
    
    