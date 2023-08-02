################################################################
# 
# Author: Mike Chen 
# From Peking university
# Last update: 2023.5.26
# 
###############################################################


import numpy as np

'''
'index': the index of agent
'K': horizon length
'h': sampling time interval
't_w': waiting time
't_c': computation time
'type': agent type, for type can be found at 'ASSP/src/planner/scripts/Dynamic'
'state': intial state
'tar': target state
'''

# agent 0:
global agent_list

agent_list = [

    {
        'index': 0,
        'K': 15,
        'h': 0.15,
        't_w': 0.18,
        't_c':0.14,
        'type':'Mini_ack',
        'state': np.array([3.05, 0.0, np.pi]),
        'tar': np.array([-3.05, 0.0, np.pi]),
    },

    {
        'index': 1,
        'K': 12,
        'h': 0.2,
        't_w': 0.28,
        't_c':0.24,
        'type':'Mini_mec',
        'state': np.array([3.05*0.707, 3.05*0.707]),
        'tar': np.array([-3.05*0.707, -3.05*0.707]),
    },

    {
        'index': 2,
        'K': 15,
        'h': 0.15,
        't_w': 0.42,
        't_c':0.32,
        'type':'Mini_4wd',
        'state': np.array([0.0, 3.05, -np.pi/2]),
        'tar': np.array([0.0, -3.05, -np.pi/2]),
    },

    {
        'index': 3,
        'K': 11,
        'h': 0.2,
        't_w': 0.34,
        't_c':0.20,
        'type':'Mini_om',
        'state': np.array([-3.05*0.707, 3.05*0.707]),
        'tar': np.array([3.05*0.707, -3.05*0.707]),
    },

    {
        'index': 4,
        'K':15,
        'h': 0.15,
        't_w': 0.20,
        't_c':0.16,
        'type':'Mini_ack',
        'state': np.array([-3.05, 0.0, 0.0]),
        'tar': np.array([3.05, 0.0, 0.0]),
    },

    {
        'index': 5,
        'K': 11,
        'h': 0.2,
        't_w': 0.28,
        't_c':0.20,
        'type':'Mini_mec',
        'state': np.array([-3.05*0.707, -3.05*0.707]),
        'tar': np.array([3.05*0.707, 3.05*0.707]),
    },

    {
        'index': 6,
        'K': 15,
        'h': 0.15,
        't_w': 0.32,
        't_c':0.24,
        'type':'Mini_4wd',
        'state': np.array([0.0, -3.05, np.pi/2]),
        'tar': np.array([0.0, 3.05, np.pi/2]),
    },

    {
        'index': 7,
        'K': 12,
        'h': 0.15,
        't_w': 0.36,
        't_c':0.32,
        'type':'Mini_tank',
        'state': np.array([3.05*0.707, -3.05*0.707,0.75*np.pi]),
        'tar': np.array([-3.05*0.707, 3.05*0.707,0.75*np.pi]),
    },

]