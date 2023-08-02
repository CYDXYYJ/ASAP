#!/usr/bin/env python
import rospy
from planner.msg import CommuTraj, ExecuteTraj
from geometry_msgs.msg import PoseStamped
import time
import copy
import numpy as np

class Neighbour():

    def __init__(self,index,j_index):

        self.state=0
        
        # own index
        self.index=index

        # the neighbour's index
        self.j_index=j_index

        # own sequence number
        self.sequence=0

        # own's beginning time of lastly planned trajectory    
        self.last_time=0

        # last protocol updating time
        self.update_time=time.time()

        # the communication delay with this neighbour
        self.delay=0.0

        self.gamma=0.0001

        # the protocol with this neighbour
        self.protocol=[]

        # the agreement list used for avoid a repeated agreement
        self.agreement_list=[]

        # the communicated trajectory with this neighbour
        self.CommuTraj=CommuTraj()

        # define subscriber and publisher with this neighbour
        self.sub=rospy.Subscriber('CommuTraj'+str(j_index)+'to'+str(self.index), CommuTraj, callback=self.response)
        self.pub=rospy.Publisher('CommuTraj'+str(self.index)+'to'+str(j_index), CommuTraj, queue_size=1)


    def update_delay(self,delay):

        self.delay=0.7*self.delay+0.3*delay

        print(self.delay)


    def update_protocol(self,OtherTraj):

        traj_start_time=time.time()-self.delay-OtherTraj.r_start
        
        start_time=max(self.last_time+self.CommuTraj.validity,traj_start_time+OtherTraj.validity)
        end_time=float('inf')

        if self.protocol!=[]:

            self.protocol[-1]['end_time']=start_time

        shapei=np.array(self.CommuTraj.shape)
        if len(shapei)>1:
            shapei=shapei.reshape(int(len(shapei)/2),2)

        shapej=np.array(OtherTraj.shape)
        if len(shapej)>1:
            shapej=shapej.reshape(int(len(shapej)/2),2)
        
        self.protocol+=[
            {   
                'start_time': start_time,
                'end_time': end_time,
                'j':{
                    'start_time': traj_start_time,
                    'x': OtherTraj.traj_x,
                    'y': OtherTraj.traj_y,
                    'theta': OtherTraj.traj_theta,
                    'duration':OtherTraj.duration,
                    'shape': shapej
                },
                'i':{
                    'start_time': self.last_time,
                    'x': self.CommuTraj.traj_x,
                    'y': self.CommuTraj.traj_y,
                    'theta': self.CommuTraj.traj_theta,
                    'duration': self.CommuTraj.duration,
                    'shape': shapei
                }
            }
        ]

        del_list=[]
        i=0
        for agreement in self.protocol:

            # the very outdated agreenment is deleted
            if self.last_time+self.CommuTraj.validity > agreement['end_time']:
                del_list+=[i]
            
            # delete the overlap agreement
            if i < len(self.protocol)-1:
                if abs(agreement['start_time']-self.protocol[i+1]['start_time'])<1e-2:
                    self.protocol[i+1]['start_time']=agreement['start_time']
                    del_list+=[i]

            i+=1

        del_list.reverse()
        for i in del_list:
            del self.protocol[i]

        self.update_time=time.time()


    def update_gamma(self,gamma):

        # gamma is an important coefficient used for deadlock resolution

        if self.gamma is None:
            self.gamma=gamma 
        else:
            self.gamma+=0.7*self.gamma+0.3*gamma


    def initiate_agreement(self,CommuTraj):
        
        # used for measuring the time delay
        self.initiate_time=time.time()

        self.pub.publish(CommuTraj)


    def response(self,OtherTraj):

        if self.state==1:

            flag=False

            if OtherTraj.initiator==False:

                agreement=str(self.index)+'-'+str(self.sequence)+' & '+str(OtherTraj.index)+'-'+str(OtherTraj.sequence)

                if not (agreement in self.agreement_list):

                    flag=True

                    self.agreement_list+=[agreement]
                    dual_agreement=str(OtherTraj.index)+'-'+str(OtherTraj.sequence)+' & '+str(self.index)+'-'+str(self.sequence)
                    self.agreement_list+=[dual_agreement] 

                    self.update_protocol(OtherTraj)

                    self.update_delay((time.time()-self.initiate_time)/2)

            else:

                agreement=str(OtherTraj.index)+'-'+str(OtherTraj.sequence)+' & '+str(self.index)+'-'+str(self.sequence)

                if not (agreement in self.agreement_list):

                    flag=True

                    self.agreement_list+=[agreement]
                    dual_agreement=str(self.index)+'-'+str(self.sequence)+' & '+str(OtherTraj.index)+'-'+str(OtherTraj.sequence)
                    self.agreement_list+=[dual_agreement]       

                    self.CommuTraj.initiator=False
                    self.CommuTraj.r_start=time.time()-self.last_time

                    self.pub.publish(self.CommuTraj)

                    self.update_protocol(OtherTraj)
                    

            if flag:     
                print('-------------------------')
                print('Agent'+str(self.index)+' reach agreement: '+agreement)
                print(' ')    


        return None


class planner():

    def __init__(self,agent):

        self.index=agent['index']

        rospy.init_node('Agent'+str(self.index), anonymous=False)
        self.Traj_pub=rospy.Publisher('Traj'+str(self.index), ExecuteTraj, queue_size=3)
        self.target_sub=rospy.Subscriber('/goal'+str(self.index), PoseStamped, callback=self.get_target,queue_size=1)

        self.t_w=agent['t_w']
        self.t_c=agent['t_c']
        self.h=agent['h']
        self.K=agent['K']
        self.type=agent['type']
        self.real_target=agent['state']
        self.target=agent['state']
        self.validity=self.t_c+self.t_w
        self.CommuTraj=CommuTraj()
        self.ExecuteTraj=ExecuteTraj()
        self.sequence=0
        self.ini_state=agent['state']
        
        traj_x=self.ini_state[0]*np.ones(self.K+1)
        traj_y=self.ini_state[1]*np.ones(self.K+1)
        self.traj=[traj_x,traj_y]

        self.time_list=np.array([0.0])


        if agent['type']=='Markanem':
            from Dynamic.Omnidirection.Markanem import markanem
            self.dynamic=markanem(self.index,self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=np.zeros(self.K+1)
        elif agent['type']=='Mini_mec':
            from Dynamic.Omnidirection.Mini_mec import mini_mec
            self.dynamic=mini_mec(self.index,self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=np.zeros(self.K+1)
        elif agent['type']=='Mini_om':
            from Dynamic.Omnidirection.Mini_om import mini_om
            self.dynamic=mini_om(self.index,self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=np.zeros(self.K+1)
        elif agent['type']=='Walle':
            from Dynamic.Unicycle.Walle import walle
            self.dynamic=walle(self.index, self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=self.ini_state[2]*np.ones(self.K+1)
        elif agent['type']=='Mini_4wd':
            from Dynamic.Unicycle.Mini_4wd import mini_4wd
            self.dynamic=mini_4wd(self.index, self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=self.ini_state[2]*np.ones(self.K+1)
        elif agent['type']=='Mini_tank':
            from Dynamic.Unicycle.Mini_tank import mini_tank
            self.dynamic=mini_tank(self.index, self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=self.ini_state[2]*np.ones(self.K+1)
        elif agent['type']=='Mini_ack':
            from Dynamic.Car.Mini_ack import mini_ack
            self.dynamic=mini_ack(self.index,self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=self.ini_state[2]*np.ones(self.K+1)
        elif agent['type']=='Formula1':
            from Dynamic.Car.Formula1 import formula1
            self.dynamic=formula1(self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=self.ini_state[2]*np.ones(self.K+1)
        elif agent['type']=='Porsche':
            from Dynamic.Car.Porsche import porsche
            self.dynamic=porsche(self.K,self.h,self.ini_state,self.target)
            self.shape=self.dynamic.shape.ravel()
            traj_theta=self.ini_state[2]*np.ones(self.K+1)
        else:
            raise Exception('Do not have this type, please define or check it')
        
        self.traj+=[traj_theta]

        # the neighbors that requires to communicate with
        self.neighbor_list=[]
        self.neighbor_index=[]

        # update trajectory for commnication and execution
        self.update_traj()

        # publish traj for low-level controler
        self.execute_traj()


    # scan avaliable neighbour in its own communication range
    def scan(self):

        topics=rospy.get_published_topics()

        for topic in topics:
            name=topic[0]
            if name[0:5]=='/Traj':
                j_index=int(name[5:])
                if not(j_index in self.neighbor_index) and j_index!=self.index:
                    self.neighbor_index+=[j_index]
                    self.neighbor_list+=[Neighbour(self.index,j_index)]

        t=time.time()

        del_list=[]
        for i in range(len(self.neighbor_list)):
            if t-self.neighbor_list[i].update_time>2.0:
                del_list+=[i]
        del_list.reverse()

        for i in del_list:
            del self.neighbor_list[i]
            # This part is not perfect that if these exists an agent who has been removed from neighbour list, once it appears again, it cannot be added once more.


    def get_target(self,msg):

        px=msg.pose.position.x
        py=msg.pose.position.y
        theta=np.arctan2(2*(msg.pose.orientation.w*msg.pose.orientation.z+msg.pose.orientation.x*msg.pose.orientation.y),\
        (1-2*(msg.pose.orientation.y*msg.pose.orientation.y+msg.pose.orientation.z*msg.pose.orientation.z)))

        if self.type=='Mini_mec' or self.type=='Mini_om' or self.type=='Walle':
            self.real_target=np.array([px,py])
        else:
            self.real_target=np.array([px,py,theta])

    # initiate agreements with others
    def initiate_agreement(self):
        
        self.CommuTraj.initiator=True 
        self.CommuTraj.r_start=time.time()-self.last_time

        for nei in self.neighbor_list:
            nei.initiate_agreement(self.CommuTraj)


    def plan_traj(self):

        from inter_aviodance import get_inter_cons
        from obstacle_aviodance import get_ob_cons

        start=time.time()
        inter_cons=get_inter_cons(self)
        print('Inter-aviodance cost:'+str(time.time()-start)+'s')
        ob_cons=get_ob_cons(self)
        
        self.dynamic.trajectory_planning(inter_cons,ob_cons)
        self.dynamic.post_processing(self.validity)
        
        # 这个地方还需要改善，因为对邻居的定义还有待改变
        # if self.neighbor_list !=[]:
        #     for nei,gamma in zip(self.neighbor_list,self.dynamic.gamma):
        #         nei.update_gamma(gamma)

        return self.dynamic.traj


    def update_traj(self):
        
        # update communication trajectory
        self.CommuTraj.index=self.index 
        self.CommuTraj.duration=self.K*self.h
        self.CommuTraj.validity=self.validity
        self.CommuTraj.sequence=self.sequence
        self.CommuTraj.traj_x=self.traj[0].tolist()
        self.CommuTraj.traj_y=self.traj[1].tolist()
        if len(self.traj)>2:
            self.CommuTraj.traj_theta=self.traj[2].tolist()
        self.CommuTraj.shape=self.shape.tolist()

        # update execution trajectory
        self.ExecuteTraj.duration=self.K*self.h
        self.ExecuteTraj.sequence=self.sequence
        self.ExecuteTraj.start_time=time.time()
        self.ExecuteTraj.type=self.type
        self.ExecuteTraj.h=self.h
        self.ExecuteTraj.traj_x=self.traj[0].tolist()
        self.ExecuteTraj.traj_y=self.traj[1].tolist()
        if len(self.traj)>2:
            self.ExecuteTraj.traj_theta=self.traj[2].tolist()


    def execute_traj(self):

        # update execution trajectory
        self.Traj_pub.publish(self.ExecuteTraj)


    def run(self,seq):


        self.sequence=seq

        self.dynamic.set_target(self.real_target)

        self.last_time=time.time()

        # update trajectory for commnication and execution
        self.update_traj()

        # publish traj for low-level controler
        self.execute_traj()

        self.scan()

        # update communication trajectory for each neighbour
        for nei in self.neighbor_list:
            nei.sequence=seq
            nei.last_time=self.last_time
            nei.CommuTraj=copy.deepcopy(self.CommuTraj)

        # begin receiving information
        for nei in self.neighbor_list:
            nei.state=1

        # initiating agreements for others 
        self.initiate_agreement()

        # waiting time for receving neighbours' information
        time.sleep(self.t_w)

        print('<=================================>')
        # stop receiving information
        for nei in self.neighbor_list:
            nei.state=0

        c_start=time.time()
        # trajectory planning
        self.traj=self.plan_traj()
        
        compute_time=time.time()-c_start
        self.time_list=np.append(self.time_list,compute_time)

        print('Computation uses: '+str(compute_time))
        print('Validity is: '+str(self.validity))
        print('====================================')
        print(' ')

        while True:
            if (time.time()-self.last_time>self.t_c+self.t_w):
                if seq>0:
                    self.validity=0.3*(time.time()-self.last_time)+0.7*self.validity
                break
            else:
                time.sleep(0.001)

        
    
def main():

    import sys,SET
    import signal

    signal.signal(signal.SIGINT, quit)                                
    signal.signal(signal.SIGTERM, quit)


    if len(sys.argv)==1:
        index=0
    else:
        index=sys.argv[1] 

    i=0
    for agent in SET.agent_list:
        if agent['index']==int(index):
            break
        i+=1

    Agent=SET.agent_list[i]

    try:
        Planner = planner(Agent)
        i=0
        while True:
            try:
                Planner.run(i)
                i+=1
            except KeyboardInterrupt:
                break 
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':

    main()