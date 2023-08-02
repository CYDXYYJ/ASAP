################################################################
# 
# Author: Mike Chen 
# From Peking university
# Last update: 2023.5.26
# 
################################################################

 
import numpy as np
from .Unicycle import unicycle

class mini_tank(unicycle):

    def __init__(self, index, K, h, ini_state, target):

        # the shape 
        shape=np.array([0.20])

        # maximum velocity
        Vmax=0.6

        # maximum accelration
        Amax=4.0

        # maximum span rate
        Omega_max=np.pi/4

        # buffer
        buffer=0.2


        super().__init__(index, K, h, ini_state, target,buffer=buffer,Vmax=Vmax,Amax=Amax,shape=shape,Omega_max=Omega_max)
