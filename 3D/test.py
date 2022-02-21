import      SET
import      datetime
from        run          import *
from        others   import *
import      numpy        as np
import pickle
import copy

def data_capture(a, b, c):
    data = {
        'pos_list': copy.copy(a),
        'position_list': copy.copy(b),
        'terminal_index_list': copy.copy(c)
    }
    return data

def initialize():
    agent_list=[]
    for i in range(SET.Num):
        agent_list+=[ uav(i,SET.ini_x[i],SET.ini_v[i],SET.target[i],SET.K) ]

    return agent_list

def PLAN( Num, ini_x, ini_v,target,r_min,epsilon,h,K,episodes):


    
    SET.initialize_set(Num, ini_x , ini_v ,target,r_min,epsilon ,h ,K,episodes)

    obj = {}

    ReachGoal=False

    episodes=SET.episodes
    
    agent_list=initialize()

    collect_data(agent_list)

    obj[0] = data_capture(SET.pos_list, SET.position_list, SET.terminal_index_list)

    # the main loop
    start =datetime.datetime.now()
    end = start

    for i in range(1,episodes+1):
        end_last=end


        obstacle_list=get_obstacle_list(agent_list,SET.Num)

        # run one step
        agent_list = run_one_step(agent_list,obstacle_list)

        # print
        end = datetime.datetime.now()
        print("Step %s have finished, running time is %s"%(i,end-end_last))

        collect_data(agent_list)

        obj[i] = data_capture(SET.pos_list, SET.position_list, SET.terminal_index_list)

        if ReachGoal:
            break

        ReachGoal=check_reach_target(agent_list)

        
        

    obj['goal'] = SET.target

    return obj,agent_list