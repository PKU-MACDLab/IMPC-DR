import numpy as np
import  matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



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
        '#2E8B57',
        '#1f77b4',
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

def plot_cube(ax,pos,ld,color):

    x, y, z = pos
    dx, dy, dz = ld

    xx = np.linspace(x-dx/2, x+dx/2, 2)
    yy = np.linspace(y-dy/2, y+dy/2, 2)
    zz = np.linspace(z-dz/2, z+dz/2, 2)

    xm, ym = np.meshgrid(xx, yy)
    ax.plot_surface(xm, ym, (z-dz/2)*np.ones((2,2)),color=color)
    ax.plot_surface(xm, ym, (z+dz/2)*np.ones((2,2)),color=color)

    ym, zm = np.meshgrid(yy, zz)
    ax.plot_surface((x-dx/2)*np.ones((2,2)), ym, zm,color=color)
    ax.plot_surface((x+dx/2)*np.ones((2,2)), ym, zm,color=color)

    xm, zm = np.meshgrid(xx, zz)
    ax.plot_surface(xm, (y-dy/2)*np.ones((2,2)), zm,color=color)
    ax.plot_surface(xm, (y+dy/2)*np.ones((2,2)), zm,color=color)


# plot trajectory
def plot_trajectory(agent_list,r_min):


    fig=plt.figure(figsize=(10,10))
    axes = fig.add_subplot(111,projection='3d')
    # axes.view_init(elev=23, azim=45)

    K=len(agent_list[0].position)-1

    for i in range(len(agent_list)):
        pos=(agent_list[i].position[K][0],agent_list[i].position[K][1],agent_list[i].position[K][2])
        
        t = np.linspace(0, np.pi * 2, 200)
        s = np.linspace(0, np.pi, 200)
        t, s = np.meshgrid(t, s)
        radius=r_min/2
        x = radius*np.cos(t) * np.sin(s) + pos[0]
        y = radius*np.sin(t) * np.sin(s) + pos[1]
        z = radius*np.cos(s) + pos[2]
        
        axes.plot_surface(x, y, z,rstride=10, cstride=10,color=color[i],zorder=1)
    
    for i in range(len(agent_list)):
        
        axes.plot(agent_list[i].position[:,0],agent_list[i].position[:,1],\
            agent_list[i].position[:,2],zorder=2,c=color[i],linewidth=4)
        
        pos=(agent_list[i].position[0][0],agent_list[i].position[0][1],agent_list[i].position[0][2])
        ld=(0.1,0.1,0.1)
        plot_cube(axes,pos,ld,color=color[i])
        # axes.scatter(agent_list[i].position[0][0],agent_list[i].position[0][1],\
        #     agent_list[i].position[0][2],marker='s',s=300,zorder=3,edgecolor='k',color=color[i])
        # axes.scatter(agent_list[i].target[0],agent_list[i].target[1],\
        #     agent_list[i].target[2],marker='d',s=400,zorder=4,edgecolor='k',color=color[i])
    
    
    axes.set_xlim(-0.5,2.5)
    axes.set_ylim(-0.5,2.5)
    axes.set_zlim(-0.6,1.6)

    axes.set_zticks([-0.5,0.0,0.5,1.0,1.5])
    

    axes.set_xlabel('x(m)')
    axes.set_ylabel('y(m)')
    axes.set_zlabel('z(m)')


    plt.savefig('trajecotry.svg')
    plt.close()