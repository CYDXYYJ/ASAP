#!/usr/bin/env
from re import I
import rospy 
from planner.msg import  ExecuteTraj, CommuTraj
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import numpy as np
import time
import copy


class object():

    def __init__(self,name) -> None:
        
        self.name=name

        self.index=int(name[5:])

        self.sub=rospy.Subscriber(name,ExecuteTraj,callback=self.saveTraj)

        self.traj=ExecuteTraj()

        self.start=time.time()

        self.update_time=time.time()

        self.type=self.traj.type

        self.obtain=False

        self.p=None

        while True:
            if self.obtain:
                break
            else:
                time.sleep(0.1)


    def saveTraj(self,traj) -> None:

        if self.type!=self.traj.type:

            self.obtain=True

        self.traj=traj

        self.start=time.time()

        self.update_time=time.time()


    def get_p(self) -> None:


        now=time.time()


        r_t=now-self.start
        h=self.traj.h

        if r_t>self.traj.duration*0.95:

            px=get_sample(self.traj.traj_x,h,self.traj.duration*0.95)
            py=get_sample(self.traj.traj_y,h,self.traj.duration*0.95)

        else:

            px=get_sample(self.traj.traj_x,h,r_t)
            py=get_sample(self.traj.traj_y,h,r_t)


        self.p=np.array([px,py])
        

def get_sample(P,h,t):

    if abs(h)<1e-5:
        return 0.0

    i=t/h
    i_c=int(np.ceil(i))
    l=len(P)

    if i_c>=l-1:
        p_c=P[-1]
    else:
        p_c=P[i_c]

    i_f=int(np.floor(i))

    if i_f>=l-1:
        return P[-1]
    else:
        p_f=P[i_f]

    return p_c*(i-i_f)+p_f*(i_c-i) 

class commu_pub():

    def __init__(self) -> None:

        rospy.init_node('commu_pub', anonymous=False)

        self.puber=rospy.Publisher('/Commu',MarkerArray,queue_size=1)

        self.markarray = MarkerArray()

        self.topic_list=[]
        self.object_list=[]
        self.commu_sub_list=[]
        self.index_list=[]

        self.pub_list=[]


    def scan(self) -> None:
        
        topics=rospy.get_published_topics()

        for topic in topics:
            name=topic[0]
            if name[0:5]=='/Traj':
                if not(name in self.topic_list):
                    self.topic_list+=[name]
                    self.object_list+=[object(name)]
                    self.index_list+=[int(name[5:])]

                    index_i=self.index_list[-1]
                    for index_j in self.index_list[:-1]:
                        name1='/CommuTraj'+str(index_i)+'to'+str(index_j)
                        name2='/CommuTraj'+str(index_j)+'to'+str(index_i)
                        self.commu_sub_list+=[rospy.Subscriber(name1,CommuTraj,callback=self.save,callback_args=[index_i,index_j])]
                        self.commu_sub_list+=[rospy.Subscriber(name2,CommuTraj,callback=self.save,callback_args=[index_j,index_i])]
    
    def query_p(self,index):

        for ob in self.object_list:
            if ob.index==index:
                return ob.p
        
        return None

    def save(self,msg,indexs):

        if msg.initiator:
            return None
        
        self.pub_list=[[time.time(),indexs[0],indexs[1]]]+self.pub_list


    def pub(self):

        t = time.time()

        self.markarray.markers=[]

        i=0
        for pub in self.pub_list:

            if t-pub[0]<0.08:

                p1 = self.query_p(pub[2])
                p2 = self.query_p(pub[1])

                if (p1 is None) or (p2 is None):
                    continue
                else:
                    i+=1

                m=Marker()
                m.header.frame_id="map"
                m.lifetime=rospy.Duration(0.3)
                m.type=Marker.ARROW
                m.id=i 
                

                m.color.a=1.0
                m.color.b=1.0
                m.color.r=187/255
                m.color.g=1.0

                m.scale.x=0.016
                m.scale.y=0.03
                m.scale.z=0.08

                p_start = Point()
                p_end = Point()
                

                p_start.x = p1[0]
                p_start.y = p1[1]

                p_end.x = p2[0]
                p_end.y = p2[1]

                m.points.append(p_start)
                m.points.append(p_end)

                self.markarray.markers.append(copy.deepcopy(m))
            
            else:

                break

        self.puber.publish(self.markarray)



    def run(self) -> None:

        self.scan()

        for ob in self.object_list:
            ob.get_p()
        self.pub()

        time.sleep(0.08)




def main():

    import signal

    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)

    try:
        vis=commu_pub()

        while True:
            vis.run()
            try:
                vis.run()
            except rospy.ROSInterruptException:
                break 
    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':
    print('hahah')
    main()