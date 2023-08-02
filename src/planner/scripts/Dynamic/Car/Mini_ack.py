from .Car import car
import numpy as np

class mini_ack(car):

    def __init__(self,index, K, h, ini_x, target):

        # the shape of this car
        shape=np.array([0.20])
        # shape=np.array([3.5])

        # maximum acc
        Amax=3.0

        Dmax=5.0

        # maximum velocity
        Vmax=1.0

        V_max=0.0

        L=0.18

        # maximum steering angle
        delta_max=25/180*np.pi
    
        buffer=0.2

        super().__init__(index, K, h, ini_x, target,shape=shape,L=L,delta_max=delta_max,Dmax=Dmax,Vmax=Vmax,V_max=V_max,Amax=Amax,buffer=buffer)