# -*- coding: utf-8 -*-
"""
Created on Sat Jul  7 18:13:03 2018

@author: sama
"""

import numpy as np
import matplotlib.pylab as plt
import bpPlotClass as bpc
import matplotlib.image as mpimg

plt.rcParams['font.family'] = 'serif'
plt.rcParams.update({'font.size': 9})

path='/home/sama/Documents/enkiSimulator/examples/build-enkiSimulator-Desktop-Debug/'
spath='./'

#plt.close("all")
mycolor = "blue"


#%%
error = np.loadtxt('{}errorData.tsv'.format(path));
integral = np.loadtxt('{}errorInteg.tsv'.format(path));
errorfig = plt.figure()
erroraxe = errorfig.add_subplot(111)
plt.plot(error, color= "black" , linestyle="-", linewidth=1)
plt.plot(integral/20, color= "grey" , linestyle="--", linewidth=1)
errorfig.savefig(spath+'error_signal', quality= 100, format='svg', bbox_inches='tight')
plt.show()


#gradient = np.loadtxt('{}gradient.tsv'.format(path));
#gradientfig = plt.figure('gradient')
#gradientaxe = gradientfig.add_subplot(111)
#plt.plot(gradient, color= mycolor , linestyle="-", linewidth=1)
#gradientfig.savefig(spath+'gradient', quality= 100, format='svg', bbox_inches='tight')
#plt.show()

#%%

#coofig=plt.figure('coordinates')
#coorddata=np.loadtxt('{}coordData.tsv'.format(path));
#img=mpimg.imread('{}cc.png'.format(path))
#imgplot = plt.imshow(img)
#plt.plot((coorddata[:,0])*4, 1000-(coorddata[:,1])*4, color= mycolor, linestyle="--", linewidth=0.5)
#coofig.savefig(spath+'coordData', quality=100, format='svg', bbox_inches='tight')
#plt.show()

#%%
#
#statRaw=np.loadtxt('{}stats.tsv'.format(path))
#stat=statRaw.astype(np.int16)
#numPredictors=stat[0]
#numLayerNeurons=stat[1::]
#
#for i in range(numPredictors):
#    predictor=bpc.predictor(i+1)
#    predictor.plotPredictor(error)
#    
#
#%%
#    
#nLayers = len(numLayerNeurons)
#wch=np.loadtxt('{}weight_distances.tsv'.format(path));# error=errordata[:,0]
#wch1=wch[:,nLayers] # only looking at the overal weight change
#
#wchfig=plt.figure('weigth change')
#axe=wchfig.add_subplot(111)
#plt.plot(wch1, color = mycolor, linestyle="-", linewidth=0.5)
#wchfig.savefig(spath+'weightchange', quality= 100, format='svg', bbox_inches='tight')
#plt.show()
#
#%%
#
#layer=bpc.layer(1) # only looking at the first layer pattern
#layer.plotLayerWeights()



