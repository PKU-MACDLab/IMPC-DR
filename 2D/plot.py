import numpy as np
import  matplotlib.pyplot as plt
from matplotlib.patches import Circle

color=[ '#1f77b4',
        '#ff7f0e', 
        '#2ca02c', 
        '#d62728',
        '#9467bd',
        '#8c564b', 
        '#e377c2',
        '#7f7f7f', 
        '#bcbd22',
        '#17becf',
        '#2F4F4F',
        '#CD5C5C',
        '#ADD8E6',
        '#663399',
        '#8FBC8F',
        '#00CED1',
        '#6A5ACD',
        '#808000',
        '#A0522D',
        '#FF4500',
        '#708090',
        '#BDB76B',
        '#FF6347',
        '#E9967A',
        '#F5DEB3',
        '#FFB6C1',
        '#556B2F',
        '#008080',
        '#7FFF00',
        '#FFA500',
        '#FF8C00',
        '#00FF7F',
        '#C0C0C0',
        '#483D8B',
        '#F08080',
        '#D3D3D3',
        '#66CDAA',
        '#FA8072',
        '#F4A460',
        '#48D1CC',
        '#8A2BE2',
        '#2E8B57']


# plot
def plot_trajectory(agent_list,r_min):

    fig=plt.figure(figsize=(10,10))
    axes = fig.add_subplot(111, aspect='equal')

    for k in range(len(agent_list[0].position)):
        for i in range(len(agent_list)):
            pos=(agent_list[i].position[k][0],agent_list[i].position[k][1])
            c=Circle(pos, radius = r_min/2.0, edgecolor='black',facecolor=color[i], zorder=1)
            axes.add_artist(c)
    
    for i in range(len(agent_list)):
        # plt.plot(agent_list[i].position[:,0],agent_list[i].position[:,1],zorder=2,c='k',linewidth=4)
        plt.scatter(agent_list[i].position[0][0],agent_list[i].position[0][1],marker='s',s=300,zorder=3,edgecolor='k',color=color[i])
        plt.scatter(agent_list[i].target[0],agent_list[i].target[1],marker='d',s=300,zorder=4,edgecolor='k',color=color[i])
    
    
    axes.set_xlim([-0.5,2.5])
    axes.set_ylim([-0.5,2.5])
    axes.set_xlabel('x(m)')
    axes.set_ylabel('y(m)')

    plt.savefig('trajecotry.svg')
    plt.close()