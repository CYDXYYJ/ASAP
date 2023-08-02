import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np
import SET

# 发布第一版八机对穿

rospy.init_node('Puber', anonymous=False)

agent_list = SET.agent_list

num = len(agent_list)


target_pub_list = []
Target_pose_list = []


for agent in agent_list:
    
    index=agent['index']
    target_pub_list += [rospy.Publisher('/goal'+str(index), PoseStamped, queue_size=2)]
    Target_pose_list += [PoseStamped()]

for i in range(num):

    pos=agent_list[i]['tar']
    Target_pose_list[i].pose.position.x=pos[0]
    Target_pose_list[i].pose.position.y=pos[1]

    if len(pos)>2:
        theta=pos[2]

    Target_pose_list[i].pose.orientation.z=np.sin(theta/2)
    Target_pose_list[i].pose.orientation.w=np.cos(theta/2)

import time

time.sleep(0.1)

for _ in range(3):
    for i in range(num):
        target_pub_list[i].publish(Target_pose_list[i])