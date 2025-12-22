# -*- coding: utf-8 -*-
"""
Created on Tue Jun 29 18:43:45 2021

@author: Andrea Bassi. Politecnico di Milano
"""

import h5py
import numpy as np


def get_h5_datasets(fname, dataset_index=0):
    """
    Finds the datasets in HDF5 file.
    Returns the dataset specified by the dataset_index.
    """
    f = h5py.File(fname,'r')
    name,shape,found = _get_h5_dataset(f, name=[], shape=[], found=0)    
    data = np.single(f[name[dataset_index]])
    f.close()
    # print('Names:', name)
    # print('Shapes:', shape)
    
    return data
   

def _get_h5_dataset(g, name, shape, found) :
    """
    Finds all the dataset in a h5 file. 
    It is operated recursively.
    Parameters
    ----------
    g : h5 file or group.
    Returns
    ----------
    name : list, locations of the datasets 
    shape : list, shape of the datasets
    found : int, number of dataset found
    """
   
    if isinstance(g,h5py.Dataset):   
        found += 1
        name.append(g.name)
        shape.append(g.shape)
        
    if isinstance(g, h5py.File) or isinstance(g, h5py.Group):
        for key,val in dict(g).items():
            
            name,shape,found = _get_h5_dataset(val,name,shape,found)
             
    return name,shape,found 


if __name__ == '__main__':
    
    fname= 'D:\\DATA\\HyperSpectral\\temp\\210629_193940_hyper.h5'

    stack = get_h5_datasets(fname,0)
    positions = get_h5_datasets(fname,1)
    print (positions)

