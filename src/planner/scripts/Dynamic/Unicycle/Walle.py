from .Unicycle import unicycle
import numpy as np

class walle(unicycle):

    def __init__(self, index, K, h, ini_state, target):

        # the shape 
        shape=np.array([1.0])

        # maximum velocity
        Vmax=2.0

        # maximum span rate
        Omega_max=np.pi/4

        # buffer
        buffer=0.02

        super().__init__(index, K, h, ini_state, target,buffer=buffer,Vmax=Vmax,shape=shape,Omega_max=Omega_max)