import SET 
from uav import *
from avoid      import *
import os
import numpy           as    np
import multiprocessing as    mp
import cvxpy           as    cp
import threading


# class MyThread(threading.Thread):

#     def __init__(self,func,args):
#         super(MyThread,self).__init__()
#         self.func = func
#         self.args = args

#     def run(self):
#         self.result = self.func(*self.args)

#     def get_result(self):
#         try:
#             return self.result  
#         except Exception:
#             return None


# def run_one_step(agent_list,obstacle_list):

#     pool = []
#     for i in range(SET.Num):
#         thread = MyThread(run_one_agent,args=[[agent_list[i],obstacle_list]])
#         pool.append(thread)
#         thread.start()

#     agent_list=[]
#     for thread in pool:
#         thread.join() 
#         agent_list.append(thread.get_result())
    
#     return agent_list


def run_one_step(agent_list,obstacle_list):
    
    pool = mp.Pool(SET.core_num)
    items=[]
    for i in range(SET.Num):
        items.append( [agent_list[i],obstacle_list] )

    # running it in parallel
    agent_list=pool.map(run_one_agent, items)
    pool.close()
    pool.join()

    return agent_list

def run_one_agent(items):

    agent=items[0]
    obstacle_list=items[1]

    # os.sched_setaffinity(0, SET.affinity[agent.index])

    agent.change_target(SET.target[agent.index])
    
    # get avoidance constraints
    [agent.cons_A,agent.cons_b,agent.cons_C,agent.rho]=\
        GET_cons(agent,obstacle_list)
    
    # running convex program
    agent.cache=run_cvxp(agent)

    # get new cost_index
    agent.post_processing()


    return agent


# run convex program of each agents
def run_cvxp(agent):
    
    state = agent.state
    cons_A = agent.cons_A
    cons_b = agent.cons_b
    cons_C = agent.cons_C
    rho = agent.rho
    u_last = agent.u
    Delta = agent.Delta
    Delta_P = 10.0*agent.Delta_P
    K = agent.K 
    D = agent.D
    VA=agent.VA
    VB = agent.VB
    VC=agent.VC
    Phi = agent.Phi
    Theta_u = agent.Theta_u
    Theta_v=agent.Theta_v
    Xi = agent.Xi
    Umax = agent.Umax
    Vmax = agent.Vmax
    epsilon = agent.epsilon
    Xi_K = agent.Xi_K
    G_p = agent.G_p
    cost_index=agent.cost_index

    # get the needed weight coefficient matrix
    Sigma=np.zeros([ D* K, D* K])
    
    for i in range(max([cost_index-1,0]), K):
        for j in range( D):
            Sigma[ D*i+j][ D*i+j]=15


    Q= VB.T @  Phi.T @ Sigma @  Phi @  VB  +  VB.T @  Phi.T @ Delta_P @  Phi @  VB
    p = 2* VB.T @  Phi.T @ Sigma @ (  Phi @  ( VA @ state + VC ) - G_p) +\
        2* VB.T @  Phi.T @ Delta_P @ Phi @  ( VA @ state + VC ) 
    

    # define the variables
    U = cp.Variable(D*K)
    E = cp.Variable(cons_C.shape[1])
    
    # objective function
    objective = cp.Minimize( cp.quad_form(U, Q ) + p.T @ U + rho @ ( E/epsilon-cp.log(E) )  )

    # control input constraint
    constraints = [  Theta_u @ U**2 <=  Umax**2]

    # velocity constraint
    constraints += [  Theta_v @ ( Xi @ ( VA @ state + VB @ U + VC))**2 <=  Vmax**2 ]

    # avoidance constraints
    constraints += [cons_b + cons_C @ E <= cons_A @  Phi @ ( VA @ state + VB @ U + VC)] 
    constraints += [ E >= 0 ]
    constraints += [ epsilon >= E ]

    # terminal constraint
    constraints += [  Xi_K @ ( VA @ state+ VB @ U + VC) == np.zeros(D) ]

    # formulate the problem
    prob = cp.Problem(objective, constraints)

    prob.solve()
     
    return [VA @ state + VB @ U.value + VC, U.value.reshape((K,D)), E.value[1:cons_C.shape[1]]]   