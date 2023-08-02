################################################################
# 
# Author: Mike Chen 
# From Peking university
# Last update: 2023.4.7
# This file is about the formulation of inter-agent collision
#  avoidance constriants in the underlying optimization.
# 
################################################################



import numpy as np


def get_inter_cons(planner):

    eta=planner.dynamic.eta
    rho_0=planner.dynamic.rho_0
    target=planner.dynamic.tractive
    K=planner.K
    h=planner.h
    neighbour_list=planner.neighbor_list
    start=planner.last_time+planner.validity


    cons=[]
    cons_T=[]
    W_inter=[]


    for nei in neighbour_list: 

        protocol=nei.protocol

        if protocol==[]:
            continue

        # for agreement in protocol:
            # print(':::::::::::')
            # print(agreement['start_time'])
            # print(agreement['end_time'])
            # print(agreement['i']['start_time'])
            # print(agreement['j']['start_time'])

        Pi=[]
        Pj=[]
        Pi_extra=[]
        Pj_extra=[]

        for k in range(K+1):
            t=k*h+start

            if t<protocol[0]['start_time']:
                continue

            i=0
            for agreement in protocol:
                if t>=agreement['start_time'] and t<agreement['end_time']:
                    break
                i+=1
            
            agreement=protocol[i]

            Pi+=[sample_point(agreement['i'],t)] 
            Pj+=[sample_point(agreement['j'],t)]

        t=K*h+start

        if t<protocol[-1]['j']['start_time']+protocol[-1]['j']['duration']:

            extra_K=(protocol[-1]['j']['start_time']+protocol[-1]['j']['duration']-t)/h
            extra_K=int(np.ceil(extra_K))

            for k in range(K+1,K+1+extra_K):
                t=start+k*h

                i=0
                for agreement in protocol:
                    if t>=agreement['start_time'] and t<agreement['end_time']:
                        break
                    i+=1

                agreement=protocol[i]

                Pi_extra+=[sample_point(agreement['i'],t)] 
                Pj_extra+=[sample_point(agreement['j'],t)]
        

        r_t=K+1-len(Pi)

        if r_t<K-1:

            for k in range(r_t,K-1):
                i=k-r_t
                pi=Pi[i+1]
                pj=Pj[i+1]
                a,b=LSP(pi,agreement['i']['shape'],pj,agreement['j']['shape'])
                c=float(pi[0:2]@a-b)
                cons+=[[k,a,b,c]]
                

            if Pi_extra!=[]:

                Pi+=Pi_extra
                Pj+=Pj_extra
                
                for k in range(K-1,len(Pi)-2):
                    i=k-r_t
                    pi=Pi[i+1]
                    pj=Pj[i+1]
                    a,b=LSP(pi,agreement['i']['shape'],pj,agreement['j']['shape'])
                    c=float(pi[0:2]@a-b)
                    cons+=[[K-1,a,b,c]]
            i+=1

        else: # 这是一种很特殊的情况一般不会出现

            if Pi_extra!=[]:

                Pi+=Pi_extra
                Pj+=Pj_extra
                
                for k in range(0,len(Pi)-1):
                    i=k
                    pi=Pi[i]
                    pj=Pj[i]
                    a,b=LSP(pi,agreement['i']['shape'],pj,agreement['j']['shape'])
                    c=float(pi[0:2]@a-b)
                    cons+=[[K-1,a,b,c]]
            i+=1

        a,b=LSP(Pi[-1],agreement['i']['shape'],Pj[-1],agreement['j']['shape'])
        c=float(pi[0:2]@a-b)
        cons_T+=[[K-1,a,b,c]]
        cons+=[[K-1,a,b,c]]
        W_inter+=[rho_0/nei.gamma]


    cons=compress(cons,K)
    cons_T=compress(cons_T)

    return [cons,cons_T,W_inter]


def compress(Cons,K=None):

    if K is None:

        if len(Cons)>10:
            c_list=[cons[3] for cons in Cons]
            IndList=np.argsort(c_list)
            Cons=[Cons[i] for i in IndList]
    
    else:

        Cons_k_list=[[] for _ in range(K)]

        for cons in Cons:
            Cons_k_list[cons[0]]+=[cons]

        new_Cons=[]

        for Cons_k in Cons_k_list:

            if len(Cons_k)>10:
                c_list=[cons[3] for cons in Cons_k]
                IndList=np.argsort(c_list)
                new_Cons+=[Cons_k[i] for i in IndList]
            else:
                new_Cons+=Cons_k

        Cons=new_Cons

    return Cons

def sample_point(traj,t):

    l=len(traj['x'])

    h=traj['duration']/(l-1)

    i=(t-traj['start_time'])/h
    i_c=int(np.ceil(i))

    if i_c>=l-1:
        x_c=traj['x'][-1]
        y_c=traj['y'][-1]
        theta_c=traj['theta'][-1]
    else:
        x_c=traj['x'][i_c]
        y_c=traj['y'][i_c]
        theta_c=traj['theta'][i_c]

    i_f=int(np.floor(i))

    if i_f>=l-1:
        x=traj['x'][-1]
        y=traj['y'][-1]
        theta=traj['theta'][-1]
        return np.array([x,y,np.cos(theta),np.sin(theta)])
    else:
        x_f=traj['x'][i_f]
        y_f=traj['y'][i_f]
        theta_f=traj['theta'][i_f]

    x=x_c*(i-i_f)+x_f*(i_c-i)
    y=y_c*(i-i_f)+y_f*(i_c-i)
    theta=theta_c*(i-i_f)+theta_f*(i_c-i)
    # 这个地方有问题，theta在-pi处可能会出问题

    return np.array([x,y,np.cos(theta),np.sin(theta)])


def LSP(pi,shapei,pj,shapej):


    con_a=len(shapei)==1
    con_b=len(shapej)==1

    if con_a and con_b:

        r_min=shapei+shapej
        agent_i=pi[0:2]
        agent_j=pj[0:2]
        p=(agent_i+agent_j)/2
        a=(agent_i-agent_j)/np.linalg.norm(agent_i-agent_j)
        b=a @ p + r_min/2

    if con_a and not con_b:
        Rj=np.array([[pj[2],-pj[3]],[pj[3],pj[2]]])
        pj=pj[0:2]+shapej@Rj.T
        pi=pi[0:2]

        dir=pi-np.mean(pj,axis=0)
        dir/=np.linalg.norm(dir)

        agent_i=pi-dir*shapei
        agent_j=pj[np.argmax(pj @ dir)]
        p=(agent_i+agent_j)/2
        a=dir/np.linalg.norm(dir)
        b=a @ p + shapei

    if not con_a and con_b:
        Ri=np.array([[pi[2],-pi[3]],[pi[3],pi[2]]])
        pi=pi[0:2]+shapei@Ri.T
        pj=pj[0:2]

        dir=np.mean(pi,axis=0)-pj
        dir/=np.linalg.norm(dir)

        agent_i=pi[np.argmin(pi @ dir)]
        agent_j=pj+dir*shapej
        p=(agent_i+agent_j)/2
        a=dir/np.linalg.norm(dir)
        b=a @ p

    if not con_a and not con_b:

        Ri=np.array([[pi[2],-pi[3]],[pi[3],pi[2]]])
        Rj=np.array([[pj[2],-pj[3]],[pj[3],pj[2]]])
        pi=pi[0:2]+shapei@Ri.T
        pj=pj[0:2]+shapej@Rj.T

        dir=np.mean(pi,axis=0)-np.mean(pj,axis=0)
        
        agent_i=pi[np.argmin(pi @ dir)]
        agent_j=pj[np.argmax(pj @ dir)]
        p=(agent_i+agent_j)/2
        a=dir/np.linalg.norm(dir)
        b=a @ p

    if np.isnan(b):
        np.savetxt('pi.txt',pi)
        np.savetxt('pj.txt',pj)
        raise Exception('LSP failed!')

    return a,b