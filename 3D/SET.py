import numpy as np


def initialize_set(NUM, INI_X , INI_V ,TARGET,R_MIN,EPSILON ,H ,KK,EPISODES):

###################
#  personal set   #
###################
    global Num     # the number of agents
    Num=NUM

    global K       # recceding horizon
    K=KK

    global h       # time step
    h=H   

    global core_num # the number of core apllied for this computing
    core_num=1 


    global episodes
    if EPISODES==None:
        episodes=15
    else:
        episodes=EPISODES

    global r_min 
    r_min=R_MIN

    global epsilon 
    epsilon=EPSILON
    
# initial set
    global ini_x   # intial position
    ini_x=INI_X

    global ini_v   # initial velocity
    ini_v=INI_V

# target position: is a variable in some condition
    global target
    target=TARGET
    
    # 这个变量用来记录所有XFD剩余MPC视界
    global terminal_index_list 
    terminal_index_list=[]
    for i in range(Num):
        terminal_index_list+=[K]
    
    # 这个变量用来记录所有XFD的过往位置
    global position_list
    position_list=ini_x

    # 这个变量用来记录XFD的预设路径用于MPC避樟
    global pos_list
    pos_list=None
