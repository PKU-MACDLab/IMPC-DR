import numpy as np 
from scipy import linalg as lg
import SET

class uav():


    # initialization
    def __init__(self,index,ini_x,ini_v,target,ini_K=11):
        
        # the index of this agent 
        self.index=index

        # initial position
        self.ini_p=ini_x

        # initial velocity
        self.ini_v=ini_v

        # target
        self.target=target

        # current position
        self.p=ini_x

        # current velocity
        self.v=ini_v

        # current state including position and velocity 
        self.state=np.append(ini_x,ini_v)

        # obstacle list which is a list including the array of position of every obstacle 
        # in future horizon
        self.obstacle_list=None

        # the length of horizon
        self.K=ini_K

        # the dimension of uav
        self.D=3

        # input
        self.u=np.zeros(self.D)

        # maximum acc
        self.Umax=2.0

        # maximum velocity
        self.Vmax=2.0

        # warning band width
        self.epsilon=SET.epsilon

        # minimum radius
        self.r_min=SET.r_min 

        # the predetermined trajectory
        self.pre_traj=np.zeros((self.K+1,self.D))

        for i in range(self.K+1):
            self.pre_traj[i]=ini_x

        self.pre_traj_list=[]

        # the list of all past position
        self.position=np.zeros((1,3))
        self.position[0]=ini_x


        # the result of cvxpy
        self.cache=None

        # a coefficient related to the objective
        self.cost_index=ini_K
        
        # the list of target 
        self.get_target_list()

        # dynamic matrix
        self.get_dynamic()

        self.get_coef_matrix()

        self.term_overlap=False
        self.term_pos=self.p
        self.E=0.0





    def post_processing(self):
        
        # get new input
        self.u=self.cache[1][0]

        
        # get new state
        for i in range(self.D):
            self.p[i]=self.cache[0][i]
            self.v[i]=self.cache[0][self.D+i]

        self.state=np.append(self.p,self.v)

        # get predeterminted trajectory
        P=self.Phi @ self.cache[0]
        P=P.reshape(( self.K, self.D))
        self.pre_traj=np.block([[P],[P[-1]]])

        self.pre_traj_list+=[self.pre_traj]

        # get new cost_index
        P = self.pre_traj
        for i in range( self.K,0,-1):
            if( np.linalg.norm( P[i]-self.target ) > 0.01 ):
                break

        self.cost_index=i


        # get terminal overlap
        term_pos=self.pre_traj[-1]

        self.E=self.cache[2]
        

        if np.linalg.norm(term_pos-self.term_pos)<0.02 and np.linalg.norm(term_pos-self.target)>0.02:
            self.term_overlap=True
        else:
            if(type(self.E) is np.ndarray):
                if (self.epsilon-self.E < 1e-4).all():
                    self.term_overlap=False
            elif self.epsilon-self.E < 1e-4:
                    self.term_overlap=False

        self.term_pos=term_pos

        # get position list
        self.position=np.block([ [self.position],[self.p] ])


        return None

    

    def get_target_list(self):

        G_p=self.target
        for i in range(1,self.K):
            G_p=np.append(G_p,self.target)
        self.G_p=G_p

        return None

    def change_target(self,Target):
        self.target=Target 
        self.get_target_list()

        return None


    def get_coef_matrix(self):
        
        D=self.D
        K=self.K

        # position matrix
        # get all position matrix
        global Phi
        Phi=np.column_stack( (np.eye(D),np.zeros((D,D))) )
        phi=Phi
        for i in range(1,K):
            Phi=lg.block_diag(Phi,phi)
        self.Phi=Phi


        # get K position matrix
        global Phi_K
        Phi_K=np.zeros((D,K*D))
        for i in range(0,D):
            Phi_K[i][K*D-D+i]=1.0
        self.Phi_K=Phi_K @ Phi

         # velocity matrix
        global Xi
        Xi=np.column_stack( (np.zeros((D,D)),np.eye(D)) )
        xi=Xi
        for i in range(1,K):
            Xi=lg.block_diag(Xi,xi)
        self.Xi=Xi


        # get K velocity matrix
        global Xi_K
        Xi_K=np.zeros((D,K*D))
        for i in range(0,D):
            Xi_K[i][K*D-D+i]=1.0
        self.Xi_K=Xi_K @ Xi
        
        # gamma this matrix is used for the maximium input control constraint 
        theta_u=np.array([1.0,1.0,3.0])
        Theta_u=theta_u
        for i in range(1,K):
            Theta_u=lg.block_diag(Theta_u,theta_u)
        self.Theta_u=Theta_u

        theta_v=np.array([1.0,1.0,2.0])
        Theta_v=theta_v
        for i in range(1,K):
            Theta_v=lg.block_diag(Theta_v,theta_v)
        self.Theta_v=Theta_v
        
        
        # control input change cost
        
        Delta=np.eye(K*D)
        for i in range(D):
            Delta[i][i]=0
        for i in range(D,K*D):
            Delta[i][i-D]=-1

        self.Delta=Delta.T @ Delta

        
        Delta_P=np.zeros((K*D,K*D))
        for i in range(1,K):
            for j in range(D):
                Delta_P[i*D+j][i*D+j]=i/K
                Delta_P[i*D+j][i*D-D+j]=-i/K
        
        self.Delta_P=Delta_P.T @ Delta_P

        return None



    def get_dynamic(self):

        K=self.K
        h=SET.h

        # system dynamic in continous time
        A=np.zeros((6,6))
        B=np.zeros((6,3))
        A[0:3,3:6]=np.eye(3)
        B[3:6,0:3]=np.eye(3)

        m=A.shape[0]

        # system dynamic
        A=np.dot(np.linalg.inv(np.eye(m)-h/2*A),(np.eye(m)+h/2*A))
        B=np.dot(np.linalg.inv(np.eye(m)-h/2*A)*h,B)

        VA=A
        for i in range(2,K+1):
            C=np.eye(m)
            for j in range(1,i+1):
                C=np.dot(C,A)
            VA=np.block([[VA],[C]])
        self.VA=VA

        VB=B
        for i in range(1,K):
                VB=np.block( [ [ np.dot( np.zeros((m,m)),B ) ],[VB] ] )
        for i in range(1,K):
            C=np.dot( matrixPow(A,i-K+1),B )
            for j in range(i-K+2,i+1):
                C=np.block([[C],[np.dot(matrixPow(A,j),B)]])
            VB=np.block([[C,VB]])
        self.VB=VB

        self.VC=np.zeros(m*K)

        return None




# the power of matrix
def matrixPow(Matrix,n):
    if(type(Matrix)==list):
        Matrix=np.array(Matrix)
    if(n==1):
        return Matrix
    elif(n==0):
        return np.eye(Matrix.shape[0])
    elif(n<0):
        return np.zeros(Matrix.shape)
    else:
        return np.matmul(Matrix,matrixPow(Matrix,n-1))



        
    