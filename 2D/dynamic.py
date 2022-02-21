# 这个文件里的内容全部是关于XFD非线性动力学模型的，不需要进行修改
import numpy as np 
import SET

def get_nomial_state_list(index):
    STATIC=SET.STATIC 
    cache=SET.cache
    K=SET.terminal_index_list[index]
    state=SET.state_list[index]

    v_list=np.zeros(K+1)
    theta_list=np.zeros(K+1)

    if(STATIC):     
        v_list[0]=state[2]
        theta_list[0]=state[3]

        v_list[1:K+1]=cache[index][1][0:K]
        theta_list[1:K+1]=cache[index][2][0:K]   
        
    else:
        v_list[0:K-1]=cache[index][1][0:K-1]
        theta_list[0:K-1]=cache[index][2][0:K-1]
        
        v_list[K-1]=cache[index][1][K-1]
        theta_list[K-1]=cache[index][2][K-1]
        v_list[K]=cache[index][1][K-1]
        theta_list[K]=cache[index][2][K-1]
    
    return [v_list,theta_list]


def get_VA(v_list,theta_list,H):

    h=SET.h
    theta_k=theta_list[H] 
    v_k=v_list[H]
    theta_k_1=theta_list[H+1]
    v_k_1=v_list[H+1]
    
    A1=np.eye(4)
    A2=np.eye(4)
    B_hat=np.zeros((4,2))
    C_hat=np.zeros((4,1))

    A1[0][2]=-h/2*np.cos(theta_k_1)
    A1[1][2]=-h/2*np.sin(theta_k_1)
    A1[0][3]=h/2*v_k_1*np.sin(theta_k_1)
    A1[1][3]=-h/2*v_k_1*np.cos(theta_k_1)

    A2[0][2]=h/2*np.cos(theta_k)
    A2[1][2]=h/2*np.sin(theta_k)
    A2[0][3]=-h/2*v_k_1*np.sin(theta_k)
    A2[1][3]=h/2*v_k_1*np.cos(theta_k)

    B_hat[2][0]=h
    B_hat[3][1]=h

    C_hat[0][0]= h/2*v_k_1*np.sin(theta_k_1)*theta_k_1 + h/2*v_k*np.sin(theta_k)*theta_k
    C_hat[1][0]=-h/2*v_k_1*np.cos(theta_k_1)*theta_k_1 - h/2*v_k*np.cos(theta_k)*theta_k

    INV_A=np.linalg.inv(A1)
    A=INV_A @ A2 
    B=INV_A @ B_hat
    C=INV_A @ C_hat

    return [A,B,C]


def get_dynamic(index):
    K=SET.terminal_index_list[index]
    [v_list,theta_list]=get_nomial_state_list(index)

    A_list=[]
    B_list=[]
    C_list=[]
    for i in range(K):
        D=get_VA(v_list,theta_list,i)
        A_list+=[D[0]]
        B_list+=[D[1]]
        C_list+=[D[2]]
    VA=np.zeros((4*K,4))
    VB=np.zeros((4*K,4*K))

    row_VA=A_list[0].copy()

    row_VB=B_list[0].copy()

    for i in range(1,K):
        row_VB=np.block( [ [row_VB,np.zeros((4,2))] ] )
    
    row_VC=C_list[0]

    VA=row_VA.copy()
    VB=row_VB.copy()
    VC=row_VC.copy()
    
    for i in range(1,K):

        row_VA=A_list[i] @ row_VA
        VA=np.block( [[VA],[row_VA]] )
        
        row_VB=A_list[i] @ row_VB
        row_VB[0:4,2*i:2*i+2]=B_list[i]
        VB=np.block( [[VB],[row_VB]] )

        row_VC=A_list[i] @ row_VC+C_list[i]
        VC=np.block( [[VC],[row_VC]] )

    return [VA,VB,VC]