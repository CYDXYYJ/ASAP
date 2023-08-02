from .Unicycle import unicycle
import numpy as np

class mini_4wd(unicycle):

    def __init__(self, index, K, h, ini_state, target):

        # the shape 
        shape=np.array([0.20])

        # maximum velocity
        Vmax=0.8

        # maximum accelration
        Amax=2.0

        # maximum span rate
        Omega_max=np.pi/3

        # buffer
        buffer=0.2
 

        super().__init__(index, K, h, ini_state, target,buffer=buffer,Vmax=Vmax,Amax=Amax,shape=shape,Omega_max=Omega_max)