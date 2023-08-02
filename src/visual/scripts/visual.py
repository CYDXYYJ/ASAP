#!/usr/bin/env
import rospy 
from planner.msg import  ExecuteTraj
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped
import numpy as np
import time

class object():

    def __init__(self,name) -> None:
        
        self.name=name

        self.index=name[5:]

        self.sub=rospy.Subscriber(name,ExecuteTraj,callback=self.saveTraj)

        self.position_pub=rospy.Publisher('vis_p'+self.index,Marker,queue_size=1)

        self.traj_pub=rospy.Publisher('vis_traj'+self.index,Path,queue_size=1)

        self.past_traj_pub=rospy.Publisher('past_traj'+self.index,Path,queue_size=1)

        self.traj=ExecuteTraj()

        self.start=time.time()

        self.vis_traj=Path()

        self.past_traj=Path()

        self.vis_traj.header.frame_id="map"

        self.past_traj.header.frame_id="map"

        self.update_time=time.time()

        self.type=self.traj.type

        self.obtain=False

        while True:
            if self.obtain:
                break
            else:
                time.sleep(0.1)



    def saveTraj(self,traj) -> None:


        if self.type!=self.traj.type:

            self.obtain=True

            self.type=self.traj.type

            if self.type=='Markanem':
                self.vis=Markanem()
            elif self.type=='Porsche':
                self.vis=Porsche()
            elif self.type=='Walle':
                self.vis=Walle()
            elif self.type=='Mini_4wd':
                self.vis=Mini_4wd()
            elif self.type=='Mini_mec':
                self.vis=Mini_mec()
            elif self.type=='Mini_om':
                self.vis=Mini_om()
            elif self.type=='Mini_ack':
                self.vis=Mini_ack()
            elif self.type=='Mini_tank':
                self.vis=Mini_tank()
            elif self.type=='Formula1':
                self.vis=Formula1()
            else:
                raise Exception('The type '+self.type+' does not exist.')

        self.traj=traj

        self.start=time.time()

        self.update_time=time.time()


    def publish(self) -> None:


        import copy

        now=time.time()

        # long time no update, delete it
        if now-self.update_time>2.0:
            self.vis_traj.poses.clear()
            self.traj_pub.publish(self.vis_traj)
            self.past_traj.poses.clear()
            self.past_traj_pub.publish(self.past_traj)
            return None

        r_t=now-self.start
        h=self.traj.h

        if r_t>self.traj.duration*0.95:

            px=get_sample(self.traj.traj_x,h,self.traj.duration*0.95)
            py=get_sample(self.traj.traj_y,h,self.traj.duration*0.95)

            p_list=np.array([[px,py,0.0],[px,py,0.0]])

        else:

            px=np.zeros(30)
            py=np.zeros(30)
            i=0
            for t in np.linspace(r_t,self.traj.duration,30):
                px[i]=get_sample(self.traj.traj_x,h,t)
                py[i]=get_sample(self.traj.traj_y,h,t)
                i+=1

            p_list=np.zeros((30,3))

            p_list[:,0]=px 
            p_list[:,1]=py

        self.vis_traj.poses.clear()
    #    def push_point(self.vis_traj.points,p_list):
        pose=PoseStamped()
        for p in p_list:
            pose.pose.position.x=p[0]
            pose.pose.position.y=p[1]
            pose.pose.position.z=p[2]
            self.vis_traj.poses.append(copy.deepcopy(pose))


        p=p_list[0]
        
        theta=get_sample(self.traj.traj_theta,self.traj.h,r_t)
        self.vis.update(p,theta)

        pose=PoseStamped()
        pose.pose.position.x=p[0]
        pose.pose.position.y=p[1]
        pose.pose.position.z=p[2]
        self.past_traj.poses.append(pose)

        self.past_traj_pub.publish(self.past_traj)
        self.traj_pub.publish(self.vis_traj)
        self.position_pub.publish(self.vis.vis_position)
        

class Walle():

    def __init__(self) -> None:
        
        # self.position_pub=rospy.Publisher('vis_walle',Marker,queue_size=10)

        self.vis_position=Marker()

        self.vis_position.color.r=203/255
        self.vis_position.color.g=133/255
        self.vis_position.color.b=63/255
        self.vis_position.color.a=1.0

        self.vis_position.header.frame_id="map"

        self.vis_position.type=Marker.MESH_RESOURCE

        self.vis_position.mesh_resource="package://visual/3Dmodel/walle.stl" 

        # self.vis_position.action=Marker.ADD   

        self.vis_position.pose.position.x=0.0
        self.vis_position.pose.position.y=0.0
        self.vis_position.pose.position.z=0.0 

        self.vis_position.scale.x=1e-2
        self.vis_position.scale.y=1e-2
        self.vis_position.scale.z=1e-2

        self.vis_position.pose.orientation.x = 0.0
        self.vis_position.pose.orientation.y = 0.0
        self.vis_position.pose.orientation.z = 0.0
        self.vis_position.pose.orientation.w = 1.0

        self.vis_position.lifetime=rospy.Duration(1.0)

    def update(self,p,theta):

        pose=self.vis_position.pose
        pose.position.x=p[0]
        pose.position.y=p[1]
        pose.position.z=p[2]
        pose.orientation.x=0.0
        pose.orientation.y=0.0    
        pose.orientation.z=np.sin(theta/2)
        pose.orientation.w=np.cos(theta/2)

class Mini_4wd():

    def __init__(self) -> None:
        
        # self.position_pub=rospy.Publisher('vis_walle',Marker,queue_size=10)

        self.vis_position=Marker()

        self.vis_position.color.r=192/255
        self.vis_position.color.g=192/255
        self.vis_position.color.b=192/255
        self.vis_position.color.a=1.0

        self.vis_position.header.frame_id="map"

        self.vis_position.type=Marker.MESH_RESOURCE

        self.vis_position.mesh_resource="package://visual/3Dmodel/mini_4wd.stl" 

        # self.vis_position.action=Marker.ADD   

        self.vis_position.pose.position.x=0.0
        self.vis_position.pose.position.y=0.0
        self.vis_position.pose.position.z=0.0 

        self.vis_position.scale.x=1e-3
        self.vis_position.scale.y=1e-3
        self.vis_position.scale.z=1e-3

        self.vis_position.pose.orientation.x = 0.0
        self.vis_position.pose.orientation.y = 0.0
        self.vis_position.pose.orientation.z = 0.0
        self.vis_position.pose.orientation.w = 1.0

        self.vis_position.lifetime=rospy.Duration(1.0)

    def update(self,p,theta):

        pose=self.vis_position.pose
        pose.position.x=p[0]
        pose.position.y=p[1]
        pose.position.z=p[2]
        pose.orientation.x=0.0
        pose.orientation.y=0.0    
        pose.orientation.z=np.sin(theta/2)
        pose.orientation.w=np.cos(theta/2)

class Mini_om():

    def __init__(self) -> None:
        
        # self.position_pub=rospy.Publisher('vis_walle',Marker,queue_size=10)

        self.vis_position=Marker()

        self.vis_position.color.r=192/255
        self.vis_position.color.g=192/255
        self.vis_position.color.b=192/255
        self.vis_position.color.a=1.0

        self.vis_position.header.frame_id="map"

        self.vis_position.type=Marker.MESH_RESOURCE

        self.vis_position.mesh_resource="package://visual/3Dmodel/mini_om.stl" 

        # self.vis_position.action=Marker.ADD   

        self.vis_position.pose.position.x=0.0
        self.vis_position.pose.position.y=0.0
        self.vis_position.pose.position.z=0.0 

        self.vis_position.scale.x=1e-3
        self.vis_position.scale.y=1e-3
        self.vis_position.scale.z=1e-3

        self.vis_position.pose.orientation.x = 0.0
        self.vis_position.pose.orientation.y = 0.0
        self.vis_position.pose.orientation.z = 0.0
        self.vis_position.pose.orientation.w = 1.0

        self.vis_position.lifetime=rospy.Duration(1.0)

    def update(self,p,theta):

        pose=self.vis_position.pose
        pose.position.x=p[0]
        pose.position.y=p[1]
        pose.position.z=p[2]
        pose.orientation.x=0.0
        pose.orientation.y=0.0    
        pose.orientation.z=np.sin(theta/2)
        pose.orientation.w=np.cos(theta/2)

class Porsche():

    def __init__(self) -> None:
        
        self.vis_position=Marker()

        self.vis_position.color.r=119/255
        self.vis_position.color.g=136/255
        self.vis_position.color.b=136/255
        self.vis_position.color.a=1.0

        self.vis_position.header.frame_id="map"

        self.vis_position.type=Marker.MESH_RESOURCE

        self.vis_position.mesh_resource="package://visual/3Dmodel/911.stl" 

        # self.vis_position.action=Marker.ADD   

        self.vis_position.pose.position.x=0.0
        self.vis_position.pose.position.y=0.0
        self.vis_position.pose.position.z=0.0 

        self.vis_position.scale.x=3.2e-2
        self.vis_position.scale.y=3.2e-2
        self.vis_position.scale.z=3.2e-2

        self.vis_position.pose.orientation.x = 0.0
        self.vis_position.pose.orientation.y = 0.0
        self.vis_position.pose.orientation.z = 0.0
        self.vis_position.pose.orientation.w = 1.0

        self.vis_position.lifetime=rospy.Duration(1.0)

    def update(self,p,theta):

        pose=self.vis_position.pose
        pose.position.x=p[0]+0.05*np.sin(theta)
        pose.position.y=p[1]-0.05*np.cos(theta)
        pose.position.z=p[2]
        pose.orientation.x=0.0
        pose.orientation.y=0.0    
        pose.orientation.z=np.sin(theta/2)
        pose.orientation.w=np.cos(theta/2)

class Formula1():

    def __init__(self) -> None:
        
        # self.position_pub=rospy.Publisher('vis_formula',Marker,queue_size=10)

        self.vis_position=Marker()

        self.vis_position.color.r=150/255
        self.vis_position.color.b=205/255
        self.vis_position.color.g=205/255
        self.vis_position.color.a=1.0

        self.vis_position.header.frame_id="map"

        self.vis_position.type=Marker.MESH_RESOURCE

        self.vis_position.mesh_resource="package://visual/3Dmodel/f175.stl" 

        # self.vis_position.action=Marker.ADD   

        self.vis_position.pose.position.x=0
        self.vis_position.pose.position.y=0
        self.vis_position.pose.position.z=0.0 

        self.vis_position.scale.x=3e-2
        self.vis_position.scale.y=3e-2
        self.vis_position.scale.z=3e-2

        self.vis_position.pose.orientation.x = 0.0
        self.vis_position.pose.orientation.y = 0.0
        self.vis_position.pose.orientation.z = 0.0
        self.vis_position.pose.orientation.w = 1.0

        self.vis_position.lifetime=rospy.Duration(1.0)

    def update(self,p,theta):

        pose=self.vis_position.pose
        pose.position.x=p[0]+0.05*np.sin(theta)
        pose.position.y=p[1]-0.05*np.cos(theta)
        pose.position.z=p[2]
        pose.orientation.x=0.0
        pose.orientation.y=0.0    
        pose.orientation.z=np.sin(theta/2)
        pose.orientation.w=np.cos(theta/2)

class Markanem():

    def __init__(self) -> None:
        
        # self.position_pub=rospy.Publisher('vis_markanem',Marker,queue_size=10)

        self.vis_position=Marker()

        self.vis_position.color.r=255/255
        self.vis_position.color.b=255/255
        self.vis_position.color.g=204/255
        self.vis_position.color.a=1.0

        self.vis_position.header.frame_id="map"

        self.vis_position.type=Marker.MESH_RESOURCE

        self.vis_position.mesh_resource="package://visual/3Dmodel/part.stl" 

        # self.vis_position.action=Marker.ADD   

        self.vis_position.pose.position.x=0
        self.vis_position.pose.position.y=0
        self.vis_position.pose.position.z=0.0 

        self.vis_position.scale.x=1e-3
        self.vis_position.scale.y=1e-3
        self.vis_position.scale.z=1e-3

        self.vis_position.pose.orientation.x = 0.0
        self.vis_position.pose.orientation.y = 0.0
        self.vis_position.pose.orientation.z = 0.0
        self.vis_position.pose.orientation.w = 1.0

        self.vis_position.lifetime=rospy.Duration(1.0)

    def update(self,p,theta):

        pose=self.vis_position.pose
        pose.position.x=p[0]
        pose.position.y=p[1]
        pose.position.z=p[2]
        pose.orientation.x=0.0
        pose.orientation.y=0.0    
        pose.orientation.z=np.sin(theta/2)
        pose.orientation.w=np.cos(theta/2)

class Mini_mec():

    def __init__(self) -> None:
        
        # self.position_pub=rospy.Publisher('vis_markanem',Marker,queue_size=10)

        self.vis_position=Marker()

        self.vis_position.color.r=153/255
        self.vis_position.color.g=153/255
        self.vis_position.color.b=255/255
        self.vis_position.color.a=1.0

        self.vis_position.header.frame_id="map"

        self.vis_position.type=Marker.MESH_RESOURCE

        self.vis_position.mesh_resource="package://visual/3Dmodel/mini_mec.stl" 

        # self.vis_position.action=Marker.ADD   

        self.vis_position.pose.position.x=0
        self.vis_position.pose.position.y=0
        self.vis_position.pose.position.z=0.0 

        self.vis_position.scale.x=1e-3
        self.vis_position.scale.y=1e-3
        self.vis_position.scale.z=1e-3

        self.vis_position.pose.orientation.x = 0.0
        self.vis_position.pose.orientation.y = 0.0
        self.vis_position.pose.orientation.z = 0.0
        self.vis_position.pose.orientation.w = 1.0

        self.vis_position.lifetime=rospy.Duration(1.0)

    def update(self,p,theta):

        pose=self.vis_position.pose
        pose.position.x=p[0]
        pose.position.y=p[1]
        pose.position.z=p[2]
        pose.orientation.x=0.0
        pose.orientation.y=0.0    
        pose.orientation.z=np.sin(theta/2)
        pose.orientation.w=np.cos(theta/2)

class Mini_ack():

    def __init__(self) -> None:
        
        # self.position_pub=rospy.Publisher('vis_markanem',Marker,queue_size=10)

        self.vis_position=Marker()

        self.vis_position.color.r=192/255
        self.vis_position.color.g=192/255
        self.vis_position.color.b=192/255
        self.vis_position.color.a=1.0

        self.vis_position.header.frame_id="map"

        self.vis_position.type=Marker.MESH_RESOURCE

        self.vis_position.mesh_resource="package://visual/3Dmodel/mini_ack.stl" 

        # self.vis_position.action=Marker.ADD   

        self.vis_position.pose.position.x=0
        self.vis_position.pose.position.y=0
        self.vis_position.pose.position.z=0.0 

        self.vis_position.scale.x=1e-3
        self.vis_position.scale.y=1e-3
        self.vis_position.scale.z=1e-3

        self.vis_position.pose.orientation.x = 0.0
        self.vis_position.pose.orientation.y = 0.0
        self.vis_position.pose.orientation.z = 0.0
        self.vis_position.pose.orientation.w = 1.0

        self.vis_position.lifetime=rospy.Duration(1.0)

    def update(self,p,theta):

        pose=self.vis_position.pose
        pose.position.x=p[0]
        pose.position.y=p[1]
        pose.position.z=p[2]
        pose.orientation.x=0.0
        pose.orientation.y=0.0    
        pose.orientation.z=np.sin(theta/2)
        pose.orientation.w=np.cos(theta/2)


class Mini_tank():

    def __init__(self) -> None:
        
        # self.position_pub=rospy.Publisher('vis_markanem',Marker,queue_size=10)

        self.vis_position=Marker()

        self.vis_position.color.r=192/255
        self.vis_position.color.g=192/255
        self.vis_position.color.b=192/255
        self.vis_position.color.a=1.0

        self.vis_position.header.frame_id="map"

        self.vis_position.type=Marker.MESH_RESOURCE

        self.vis_position.mesh_resource="package://visual/3Dmodel/mini_tank.stl" 

        # self.vis_position.action=Marker.ADD   

        self.vis_position.pose.position.x=0
        self.vis_position.pose.position.y=0
        self.vis_position.pose.position.z=0.0 

        self.vis_position.scale.x=1e-3
        self.vis_position.scale.y=1e-3
        self.vis_position.scale.z=1e-3

        self.vis_position.pose.orientation.x = 0.0
        self.vis_position.pose.orientation.y = 0.0
        self.vis_position.pose.orientation.z = 0.0
        self.vis_position.pose.orientation.w = 1.0

        self.vis_position.lifetime=rospy.Duration(1.0)

    def update(self,p,theta):

        pose=self.vis_position.pose
        pose.position.x=p[0]
        pose.position.y=p[1]
        pose.position.z=p[2]
        pose.orientation.x=0.0
        pose.orientation.y=0.0    
        pose.orientation.z=np.sin(theta/2)
        pose.orientation.w=np.cos(theta/2)


class Drone():

    def __init__(self) -> None:
        
        # self.position_pub=rospy.Publisher('vis_drone',Marker,queue_size=10)

        self.vis_position=Marker()

        self.vis_position.color.r=150/255
        self.vis_position.color.b=205/255
        self.vis_position.color.g=205/255
        self.vis_position.color.a=1.0

        self.vis_position.header.frame_id="map"

        self.vis_position.type=Marker.MESH_RESOURCE

        self.vis_position.mesh_resource="package://visual/3Dmodel/drone.stl" 

        # self.vis_position.action=Marker.ADD   

        self.vis_position.pose.position.x=0.0
        self.vis_position.pose.position.y=0.0
        self.vis_position.pose.position.z=0.0

        self.vis_position.scale.x=0.2e-2
        self.vis_position.scale.y=0.2e-2
        self.vis_position.scale.z=0.2e-2

        self.vis_position.pose.orientation.x = 0.0
        self.vis_position.pose.orientation.y = 0.0
        self.vis_position.pose.orientation.z = 0.0
        self.vis_position.pose.orientation.w = 1.0

        self.vis_position.lifetime=rospy.Duration(1.0)

class F22():

    def __init__(self) -> None:
        
        # self.position_pub=rospy.Publisher('vis_F22',Marker,queue_size=10)

        self.vis_position=Marker()

        self.vis_position.color.r=220/255
        self.vis_position.color.b=220/255
        self.vis_position.color.g=220/255
        self.vis_position.color.a=1.0

        self.vis_position.header.frame_id="map"

        self.vis_position.type=Marker.MESH_RESOURCE

        self.vis_position.mesh_resource="package://visual/3Dmodel/F22.stl" 

        # self.vis_position.action=Marker.ADD   

        self.vis_position.pose.position.x=0.0
        self.vis_position.pose.position.y=0.0
        self.vis_position.pose.position.z=0.0

        self.vis_position.scale.x=1e-1
        self.vis_position.scale.y=1e-1
        self.vis_position.scale.z=1e-1

        self.vis_position.pose.orientation.x = 0.0
        self.vis_position.pose.orientation.y = 0.0
        self.vis_position.pose.orientation.z = 0.0
        self.vis_position.pose.orientation.w = 1.0

        self.vis_position.lifetime=rospy.Duration(1.0)


class rviz_pub():

    def __init__(self) -> None:

        rospy.init_node('vis_pub', anonymous=False)
        self.topic_list=[]
        self.object_list=[]


    def scan(self) -> None:
        
        topics=rospy.get_published_topics()

        for topic in topics:
            name=topic[0]
            if name[0:5]=='/Traj':
                if not(name in self.topic_list):
                    self.topic_list+=[name]
                    self.object_list+=[object(name)]


    def run(self) -> None:

        self.scan()

        for ob in self.object_list:
            ob.publish()

        time.sleep(0.08)
        

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



def main():

    import signal

    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)

    try:
        vis=rviz_pub()

        while True:
            vis.run()
            try:
                vis.run()
            except rospy.ROSInterruptException:
                break 
    except rospy.ROSInterruptException:
        pass



if __name__ == '__main__':

    main()