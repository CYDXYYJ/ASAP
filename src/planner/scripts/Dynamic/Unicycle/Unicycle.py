################################################################
# 
# Author: Mike Chen 
# From Peking university
# Last update: 2023.5.24
# 
################################################################



from sre_parse import State
import numpy as np
from acados_template import AcadosModel, AcadosOcpSolver, AcadosOcp
from casadi import SX, vertcat, sin, cos
import scipy.linalg

class unicycle():

    # initialization
    def __init__(self,
        index,
        K,
        h,
        ini_state,
        target,
        Vmax,
        Amax,
        Omega_max,
        shape,
        buffer
        ):

        ##############################
        ####### key cofficient #######
        ##############################

        self.index=index

        # the length of horizon
        self.K=K

        self.h=h

        self.Vmax=Vmax

        self.Amax=Amax

        self.Omega_max=Omega_max

        self.shape=shape

        if len(shape)==1:
            self.circle=True
        else:
            self.circle=False
            self.shape=shape.reshape(int(len(shape)/2),2)

        self.buffer=buffer

        #####################
        ####### state #######
        #####################

        # initial satte
        self.ini_p=ini_state[0:2].copy()

        # position
        self.p=ini_state[0:2].copy()

        # velocity
        self.v=0.0

        # current state         
        self.state=np.append(ini_state,self.v) 

        ##########################
        ####### Trajectory #######
        ##########################

        # target position
        self.target=target.copy()

        # the predetermined trajectory
        self.pre_traj=np.zeros((self.K+1,2))

        for i in range(self.K+1):
            self.pre_traj[i]=self.p.copy()

        # tractive position
        self.tractive=target.copy()


        # the list of all time's 
        self.pre_traj_list=[]


        #######################
        ####### Dealock #######
        #######################

        self.rho_0=20.0

        # warning band width
        self.epsilon=0.1

        self.eta=0.0

        self.term_overlap=False

        self.term_last_p=self.p.copy()

        self.term_overlap_again=False

        self.warning=None
        
        ###############################
        ####### data collection #######
        ###############################
        
        self.data=np.zeros(len(self.state))
        self.data=np.block([[self.data],\
            [self.state],[np.append(self.target,0.0)],[-9999999*np.ones(len(self.state))]])

        
        ###############################
        ######### optimization ########
        ###############################

        self.setup_nlp()


    # run convex program of each agents
    def trajectory_planning(self,Inter_cons,Ob_cons):

        ####### dynamic related #######

        state=self.state
        tractive=self.tractive.copy()
        # buffer=self.buffer
        K=self.K
        next_state=self.next_state

        ####### functional constraints related #######
        inter_cons = Inter_cons[0]
        inter_cons_T = Inter_cons[1]
        ob_cons = Ob_cons


        C_list=[np.zeros((10,4)) for _ in range(K)]
        lg_list=[-np.ones(10) for _ in range(K)]
        count_list=[0 for _ in range(K)]

        delta_yref=np.zeros((K,3))

        for cons in inter_cons+ob_cons:
            k=cons[0]
            count=count_list[k]
            if count<10:

                C_list[k][count]=np.block([cons[1],np.zeros(2)])
                lg_list[k][count]=cons[2] 
                count_list[k]+=1

        dir=tractive[0:2]-next_state[K][0:2]
        dir/=np.linalg.norm(dir)

        delta_yref=np.zeros(2)
        for cons in inter_cons_T:

            outside=cons[1] @ next_state[K][0:2]-cons[2]

            if outside<self.buffer:

                sin=dir[0]*cons[1][1]-dir[1]*cons[1][0]

                dis=0.004/(0.001+max(0,outside))-0.004/(0.001+self.buffer)
                delta_yref-=sin*dis*cons[1]
        
        
        ##############################
        ###### nolinear program ######
        ##############################


        # objective
        tractive[0:2]+=delta_yref
        print("delta_yref is: "+str(delta_yref))

        for i in range(K,0,-1):
            if (next_state[i][0:3]-tractive) @ (self.Q @ (next_state[i][0:3]-tractive))>0.5:
                break
        cost_index=i

        W1 = scipy.linalg.block_diag(self.Q,self.R)
        W2 = scipy.linalg.block_diag(10*self.Q,self.R)
        
        yref=np.block([tractive,np.zeros(2)])

        for i in range(1,cost_index):
            self.acados_ocp_solver.cost_set(i,"yref",yref)
            self.acados_ocp_solver.cost_set(i,"W",W1)

        for i in range(cost_index,K):
            self.acados_ocp_solver.cost_set(i,"yref",yref)
            self.acados_ocp_solver.cost_set(i,"W",W2)

        self.acados_ocp_solver.cost_set(K,"yref",tractive)
        self.acados_ocp_solver.cost_set(K,"W",10*self.Q)

   
        # collision aviodance constriants
        if self.circle:
            for k in range(K):
                self.acados_ocp_solver.constraints_set(k+1,"C",C_list[k],api='new')
                self.acados_ocp_solver.constraints_set(k+1,"lg",lg_list[k])
                self.acados_ocp_solver.constraints_set(k+1,"ug",1e10*np.ones(10))
        else:
            pass
            # for S in self.S_list:
            #     cos_S=S[0]
            #     sin_S=S[1]
            #     opti.subject_to(buffer-buff+cons_B<=cons_A @ ( self.Phi @ opt_states + cos_S @ ca.cos(theta) + sin_S @ ca.sin(theta) ))

        # init_condition constraints
        self.acados_ocp_solver.set(0,"lbx",state)
        self.acados_ocp_solver.set(0,"ubx",state)

        # initial guess
        for k in range(K + 1):
            self.acados_ocp_solver.set(k, "x", self.next_state[k].copy())
        for k in range(K):
            self.acados_ocp_solver.set(k, "u", self.u0[k].copy())

        # solve
        status = self.acados_ocp_solver.solve()
        # self.acados_ocp_solver.print_statistics()
        if status in [0,2]:
            opt_states=np.zeros((K+1,4))
            opt_controls=np.zeros((K,2))
            for k in range(K + 1):
                opt_states[k] = self.acados_ocp_solver.get(k, "x")
            for k in range(K):
                opt_controls[k] = self.acados_ocp_solver.get(k, "u")
        else:
            opt_states=self.next_state.copy()
            opt_controls=self.u0.copy()
            self.acados_ocp_solver.reset()

        self.cache = [opt_states,opt_controls]


    def setup_nlp(self):

        self.Q = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, .1]])
        self.R = np.array([[0.5, 0.0], [0.0, 2.0]])

        self.next_state=np.zeros((self.K+1, 4))
        for i in range(self.K+1):
            self.next_state[i]=self.state.copy()
        
        self.u0=np.zeros((self.K, 2))


        if not self.circle:
            self.S_list=[]
            for s in self.shape:
            # for s in np.array([3.6,0.7,3.6,-0.7,-0.5,-0.7,-0.5,0.7]).reshape((4,2)):
                cos_S=np.zeros((2*self.K,self.K))
                sin_S=np.zeros((2*self.K,self.K))
                for i in range(self.K):
                    cos_S[2*i,i]=s[0]
                    cos_S[2*i+1,i]=s[1]
                    sin_S[2*i,i]=-s[1]
                    sin_S[2*i+1,i]=s[0]
            
                self.S_list+=[[cos_S,sin_S]]
        
        # # create solvers
        ocp = self.create_ocp_solver_description()
        self.acados_ocp_solver = AcadosOcpSolver(
            ocp, json_file="acados_code/agent"+str(self.index)+ "/configure.json"
        )


    @staticmethod
    def export_robot_model(index) -> AcadosModel:

        model_name = "agent"+str(index)

        # set up states & controls
        x = SX.sym("x")
        y = SX.sym("y")
        theta = SX.sym("theta")
        v = SX.sym("v")
        x = vertcat(x, y, theta, v)

        a = SX.sym("a")
        omega = SX.sym("omega")
        u = vertcat(a, omega)

        # xdot
        x_dot = SX.sym("x_dot")
        y_dot = SX.sym("y_dot")
        theta_dot = SX.sym("theta_dot")
        v_dot = SX.sym("v_dot")
        xdot = vertcat(x_dot, y_dot, theta_dot, v_dot)

        # dynamics
        f_expl = vertcat(v * cos(theta), v * sin(theta), omega, a)
        f_impl = xdot - f_expl

        model = AcadosModel()

        model.f_impl_expr = f_impl
        model.f_expl_expr = f_expl
        model.x = x
        model.xdot = xdot
        model.u = u
        model.name = model_name

        return model   


    def create_ocp_solver_description(self) -> AcadosOcp:

        # create ocp object to formulate the OCP
        ocp = AcadosOcp()

        model = self.export_robot_model(self.index)
        ocp.model = model

        # set dimensions
        ocp.dims.N = self.K

        # set cost
        ocp.cost.cost_type = "LINEAR_LS"
        ocp.cost.cost_type_e = "LINEAR_LS"

        Vx = np.zeros((5,4))
        Vx[0:3,0:3] = np.eye(3)
        Vu = np.zeros((5,2))
        Vu[3:5,:] = np.eye(2)
        ocp.cost.Vx = Vx 
        ocp.cost.Vu = Vu
        ocp.cost.W = np.eye(5)
        ocp.cost.yref = np.zeros(5)

        Vx_e = np.zeros((3,4))
        Vx_e[0:3,0:3] = np.eye(3)
        ocp.cost.Vx_e = Vx_e
        ocp.cost.W_e = np.eye(3)
        ocp.cost.yref_e = np.zeros(3)

        # set constraints
        ocp.constraints.x0 = self.state

        ocp.constraints.lbu = -np.array([self.Amax,self.Omega_max])
        ocp.constraints.ubu = np.array([self.Amax,self.Omega_max])
        ocp.constraints.idxbu = np.array([0,1])

        ocp.constraints.lbx = -np.array([0.0])
        ocp.constraints.ubx = np.array([self.Vmax])
        ocp.constraints.idxbx = np.array([3])

        ocp.constraints.C = np.zeros((10,4))
        ocp.constraints.C_e = np.zeros((10,4))
        ocp.constraints.D = np.zeros((10,2))

        ocp.constraints.lg = np.zeros(10)
        ocp.constraints.ug = np.zeros(10)
        ocp.constraints.lg_e = np.zeros(10)
        ocp.constraints.ug_e = np.zeros(10)

        # set options
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # FULL_CONDENSING_QPOASES
        ocp.solver_options.hessian_approx = "EXACT"  # 'GAUSS_NEWTON', 'EXACT'
        ocp.solver_options.integrator_type = "IRK"
        ocp.solver_options.nlp_solver_type = "SQP"  # SQP_RTI, SQP
        ocp.solver_options.nlp_solver_max_iter = 10
        ocp.solver_options.qp_solver_iter_max = 10
        ocp.solver_options.print_level = 0
        ocp.solver_options.tol = 1e-4

        # set prediction horizon
        ocp.solver_options.tf = self.K*self.h

        # export directory
        ocp.code_export_directory = "acados_code/agent"+str(self.index)

        return ocp


    def post_processing(self,vadility):

        self.output=self.cache[0]
        self.input=self.cache[1]

        # get predeterminted trajectory
        self.pre_traj=self.output[:,0:2]

        # get the execution trajectory
        self.traj=self.get_traj()  

        # get new state
        self.state=get_sample(P=self.output,h=self.h,t=vadility)

        # update eta for deadlock resolution
        # self.update_eta()

        # data collection
        self.data=np.block([[self.data],[np.append(self.tractive,0.0)],[self.state],\
        [-7777777*np.ones(len(self.state))],[self.output],[-9999999*np.ones( len(self.state) )]])

        next_state=np.zeros((self.K+1,4))
        self.u0=np.zeros((self.K,2))

        for i,t in zip(range(self.K+1),vadility+self.h*np.arange(self.K+1)):
            next_state[i]=get_sample(self.output,self.h,t)

        for i, t in zip(range(self.K),vadility+self.h*np.arange(self.K)):
            self.u0[i]=get_sample(self.input,self.h,t)

        self.next_state = next_state

        return None


    # transform the predetermined trajectory to the 7-th order polynomial
    def get_traj(self):    

        return [self.output[:,0],self.output[:,1],self.output[:,2]]


    # update eta for deadlock resolution
    def update_eta(self):

        term_p=self.pre_traj[-1].copy()
         
        term_second_p=self.pre_traj[-2].copy()

        if self.term_overlap:
            
            condition_a=np.linalg.norm(term_p-self.term_last_p)<0.001
            condition_b=np.linalg.norm(term_p-term_second_p)<0.005
            condition_c=np.linalg.norm(term_p-self.tractive)>0.02

            if condition_a and condition_b and condition_c:
                self.term_overlap_again=True

        else:

            condition_a=np.linalg.norm(term_p-self.term_last_p)<0.015
            condition_b=np.linalg.norm(term_p-term_second_p)<0.02
            condition_c=np.linalg.norm(term_p-self.tractive)>0.02

            if condition_a and condition_b and condition_c:
                self.term_overlap=True

        flag=False

        if self.warning is None:

            flag=True 
        else:
            if(type(self.warning) is np.ndarray):
                if (self.epsilon-self.warning < 1e-3).all():
                    flag=True
            elif self.epsilon-self.warning < 1e-3:
                    flag=True
        
        if flag:
            self.term_overlap=False
            self.term_overlap_again=False
            self.eta=0.0

        if self.term_overlap_again and self.eta < 4.0: 
            self.term_overlap_again=False
            self.eta += 0.3

        if self.term_overlap is True and self.eta <1.0:
            self.eta=1.0
           
        self.term_last_p=term_p.copy()

    def set_target(self,target):
        
        self.target=target.copy()

        self.tractive=target.copy()
    
def get_sample(P,h,t):

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