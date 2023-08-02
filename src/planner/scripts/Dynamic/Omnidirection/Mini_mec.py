################################################################
# 
# Author: Mike Chen 
# From Peking university
# Last update: 2023.5.26
# 
################################################################


import numpy as np
from .Omnidirection import omnidirection

class mini_mec(omnidirection):

    def __init__(self, index, K, h, ini_p, target):

        # the shape of this agent
        shape=np.array([0.20])

        # maximum acc
        Umax=2.0

        # maximum velocity
        Vxmax=0.8

        Vymax=0.6

        buffer=0.2

        super().__init__(index, K, h, ini_p, target,shape=shape,Vxmax=Vxmax,Vymax=Vymax,buffer=buffer,Umax=Umax)
