################################################################
# 
# Author: Mike Chen 
# From Peking university
# Last update: 2023.3.17
# 
################################################################


import numpy as np
from .Omnidirection import omnidirection

class markanem(omnidirection):

    def __init__(self, index, K, h, ini_p, target):

        # the shape of this agent
        shape=np.array([0.2])

        # maximum acc
        Umax=1.5

        # maximum velocity
        Vxmax=1.0

        Vymax=1.0


        buffer=0.1

        super().__init__(index, K, h, ini_p, target,shape=shape,Vxmax=Vxmax,Vymax=Vymax,buffer=buffer,Umax=Umax)
