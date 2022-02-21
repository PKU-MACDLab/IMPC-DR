import numpy as np
from numpy.lib.arraypad import _set_reflect_both
import SET
import time


def get_obstacle_list(agent_list,Num):

    obstacle_list=[]
    for i in range(Num):
        obstacle_list+=[agent_list[i].pre_traj]

    return obstacle_list

def collect_data(agent_list):
    SET.pos_list=[]
    SET.position_list=[]
    SET.terminal_index_list=[]
    
    for agent in agent_list:
        SET.pos_list+=[agent.pre_traj]
        SET.position_list+=[agent.position]
        SET.terminal_index_list+=[agent.cost_index]
    

def check_reach_target(agent_list):

    REACHGOAL=True 

    for agent in agent_list:
        if agent.cost_index > 1:
            REACHGOAL=False
        
    return REACHGOAL

def check_deadlock(agen_list):

    deadlock = False 

    for agent in agen_list:
        if agent.term_overlap:
            deadlock=True
            break 

    return deadlock


        
def checkresolution(agent_list):
    for agent in agent_list:
        if agent.term_overlap:
            return True

    return False





