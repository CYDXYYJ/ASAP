from .Car import car
import numpy as np

class porsche(car):

    def __init__(self, K, h, ini_x, target):

        # the shape of this car
        shape=np.array([3.3,1.0,3.3,-1.0,-1.0,-1.0,-1.0,1.0])

        # maximum acc
        Amax=2.0

        Dmax=4.0

        # maximum velocity
        Vmax=5.0

        V_max=1.0

        L=2.3

        # maximum steering angle
        delta_max=33.5/180*np.pi
    
        buffer=0.02

        super().__init__(K, h, ini_x, target,shape=shape,L=L,delta_max=delta_max,Dmax=Dmax,Vmax=Vmax,V_max=V_max,Amax=Amax,buffer=buffer)