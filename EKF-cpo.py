#Extended Kalman Filter
import numpy as np
import math
import quat
from numpy.linalg import inv

vel = []
pos = []

class EKF:
    Mned_tp1_tp1_prev = np.zeros((3,3))
    P_tp1_t_prev = np.zeros((15,15))
    n_p_prev = np.zeros(15)
    K_tp1_prev = np.zeros((15,7))
    n_z_prev = np.zeros(7)
    PHI_prev = np.zeros((15,15))
    delX_tp1_tp1_prev = np.zeros(15)
    v_tp1_tp1_prev = np.zeros(3)[None,:]
    p_tp1_tp1_prev = np.zeros(3)[None,:]
    Mone_zero = np.zeros((3,3))
    m_tp1 = np.zeros(7)
    sum = 0

    

    def Extended_Kalman_Filter(self,deltp1,del_vel,ab,w,flag,del_t,n_p,n_z,alpha_tp1, beta_tp1, Y_tp1):

    #m t+1
        
        if(flag ==1):
            EKF.m_tp1[0:1] = deltp1
            EKF.m_tp1[1:4] = del_vel
            EKF.m_tp1[4:7] = EKF.delX_tp1_tp1_prev[9:12]
        
        wavg = (w[0]+w[1]+w[2])/3.0


        
    #update equation 34A
        w = w - EKF.delX_tp1_tp1_prev[3:6]
    # update acceleration equation 34B
        ab_prime1 = (ab - EKF.delX_tp1_tp1_prev[12:15])
        ab_prime = ab_prime1[None,:]
    #calculating Mned equation 35 36 37
   #flag=1 means Stance Phase +
        if 1 is 1:
            Mned_tp1_t=np.array([[math.cos(Y_tp1)*math.cos(beta_tp1),(math.cos(Y_tp1)*math.sin(alpha_tp1)*math.sin(beta_tp1))-(math.cos(alpha_tp1)*math.sin(Y_tp1)),(math.sin(alpha_tp1)*math.sin(Y_tp1))+(math.cos(alpha_tp1)*math.cos(Y_tp1)*math.sin(beta_tp1))],
                       [(math.cos(beta_tp1)*math.sin(Y_tp1)),(math.cos(alpha_tp1)*math.cos(Y_tp1))+(math.sin(alpha_tp1)*math.sin(Y_tp1)*math.sin(beta_tp1)),(math.cos(alpha_tp1)*math.sin(Y_tp1)*math.sin(beta_tp1))-(math.cos(Y_tp1)*math.sin(alpha_tp1))],
                       [(-1)*(math.sin(beta_tp1)),math.cos(beta_tp1)*math.sin(alpha_tp1),math.cos(alpha_tp1)*math.cos(beta_tp1)]])
            EKF.Mone_zero = Mned_tp1_t
            #print("------------------------------------------------------------")               
        """else:
            del_IpOmegaxdelt=np.array([[2,((-1)*(wavg))*del_t,wavg*del_t],
                              [wavg*del_t,2,((-1)*(wavg))*del_t],
                              [((-1)*(wavg))*del_t,wavg*del_t,2]])
            #del_IpOmegaxdelt=np.array([[2,((-1)*(w[2]))*del_t,w[1]*del_t],
            #                  [w[2]*del_t,2,((-1)*(w[0]))*del_t],
            #                  [((-1)*(w[1]))*del_t,w[0]*del_t,2]])
            del_ImOmegaxdelt_inv = inv(del_IpOmegaxdelt.T)
            Mned_tp1_t_term = np.matmul(EKF.Mned_tp1_tp1_prev,del_IpOmegaxdelt)
            Mned_tp1_t=np.matmul(Mned_tp1_t_term,del_ImOmegaxdelt_inv)
            #print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")"""

   #calculating S matrix 32
        
        an_prime=np.matmul(Mned_tp1_t,ab_prime1)

        S = np.array([[0,(-1)*(an_prime[2]),an_prime[1]],[an_prime[2],0,(-1)*(an_prime[0])],[(-1)*(an_prime[1]),an_prime[0],0]])


    #calculating  31

        PHI = np.identity(15)
        PHI[0:3,3:6] = np.multiply(del_t,Mned_tp1_t)
        PHI[6:9,9:12] = np.multiply(del_t,np.identity(3))
        PHI[6:9,12:15] = np.multiply(-1*(del_t*del_t)/2,S)
        PHI[9:12,0:3] = np.multiply(-1*del_t,S)
        PHI[9:12,12:15] = np.multiply(del_t,Mned_tp1_t)
        PHI[12:15,12:15] =  np.multiply(-1*del_t,S)

    #delXt+1|t 30
    
        term = np.matmul(PHI,EKF.delX_tp1_tp1_prev)
        delX_tp1_t = term + EKF.n_p_prev

    #H Matrix 39
        H = np.zeros((7,15))
        H[0:4,2:6] = np.identity(4)
        H[4:7,9:12] = np.identity(3)


    #R_tp1 
        NzCopy = n_z
        R_tp1_term = np.matmul(n_z[:,None], NzCopy[None,:])
        R_tp1 = np.cov(R_tp1_term)
        #print R_tp1.shape

    


    #eqn 43
        NzPrevCopy = EKF.n_z_prev
        R_t_term = np.matmul(EKF.n_z_prev[:,None], NzPrevCopy[None,:])
        R_t = np.cov(R_t_term)
        if(flag == 1):
            P_t_t = np.zeros((15,15))
            P_t_t[3:6,3:6] = np.identity(3)
            P_t_t[12:15,12:15] = np.identity(3)
            P_t_t = np.multiply(0.01,P_t_t)

        else:
            P_t_t_term1 = np.matmul(EKF.K_tp1_prev,R_t)
            P_t_t_term2 = np.matmul(P_t_t_term1,EKF.K_tp1_prev.T)
            P_t_t_term3 = np.matmul(EKF.K_tp1_prev,H)
            I_15x15 = np.identity(15)
            P_t_t_term4 = I_15x15 - P_t_t_term3
            P_t_t_term5 = np.matmul(P_t_t_term4,EKF.P_tp1_t_prev)
            P_t_t_term6 = np.matmul(P_t_t_term5,P_t_t_term4.T)
            P_t_t = P_t_t_term6 + P_t_t_term2


    #eqn 42
        P_tp1_t_term1 = np.matmul(EKF.PHI_prev,P_t_t)
        P_tp1_t_term2 = np.matmul(P_tp1_t_term1,EKF.PHI_prev.T)
        Q_t = np.cov(np.matmul(EKF.n_p_prev[:,None],EKF.n_p_prev[None,:]))
        P_tp1_t = P_tp1_t_term2 + Q_t


    #eqn 41
        
        K_tp1_term1 = np.matmul(P_tp1_t,H.T)
        K_tp1_term2 = np.matmul(H,P_tp1_t)
        K_tp1_term3 = np.matmul(K_tp1_term2,H.T)
        K_tp1_term4 = K_tp1_term3 + R_tp1
        try:
            K_tp1_term4_inv = inv(K_tp1_term4)
        except np.linalg.linalg.LinAlgError as err:
            if 'Singular matrix' in err.message:
                K_tp1_term4_inv = np.identity(7)
            else:
                raise
            
        
        K_tp1 = np.matmul(K_tp1_term1,K_tp1_term4_inv)

    #eqn 38
        delX_tp1_tp1_term1 = np.matmul(H,delX_tp1_t)
        delX_tp1_tp1_term2 = EKF.m_tp1 - delX_tp1_tp1_term1
        delX_tp1_tp1_term3 = np.matmul(K_tp1,delX_tp1_tp1_term2)
        delX_tp1_tp1 = delX_tp1_t + delX_tp1_tp1_term3

    #eqn40
        z_tp1_term1 = np.matmul(H,delX_tp1_tp1)
        z_tp1 = z_tp1_term1 + n_z

    #eqn44
        #a_e_tp1_term1 = np.matmul(EKF.Mone_zero,ab_prime1)
        a_e_tp1_term1 = np.matmul(Mned_tp1_t,ab_prime1)
        g_e = np.array([0,0,9.833])
        #print(g_e.shape)
        a_e_tp1 = a_e_tp1_term1 - g_e[None,:]
        EKF.sum = EKF.sum + a_e_tp1_term1
        #a_e_tp1 = ab_prime1 - g_e[None,:]

    #eqn45
        v_tp1_t = EKF.v_tp1_tp1_prev + np.multiply(del_t,a_e_tp1)
    #eqn46
        p_tp1_t = EKF.p_tp1_tp1_prev + np.multiply(del_t,v_tp1_t)
    
    
    #final update
        v_tp1_tp1 = v_tp1_t - delX_tp1_tp1[9:12]
        #print("wowowowowowowo",np.shape(v_tp1_tp1))
        p_tp1_tp1 = p_tp1_t - delX_tp1_tp1[6:9]

    #Mned t+1|t+1
        """del_Imphixdelt=np.array([[2,((-1)*EKF.delX_tp1_tp1_prev[2])*del_t,EKF.delX_tp1_tp1_prev[1]*del_t],
                              [EKF.delX_tp1_tp1_prev[2]*del_t,2,((-1)*EKF.delX_tp1_tp1_prev[0])*del_t],
                              [((-1)*EKF.delX_tp1_tp1_prev[1])*del_t,EKF.delX_tp1_tp1_prev[0]*del_t,2]])
        del_Imphixdelt_inv = inv(del_Imphixdelt)
        Mned_tp1_tp1_term = np.matmul(del_Imphixdelt.T,del_Imphixdelt_inv)
        Mned_tp1_tp1 =np.matmul(Mned_tp1_tp1_term,Mned_tp1_t)"""
        Mned_tp1_tp1 = Mned_tp1_t
        

        #print("Velocity",v_tp1_t)
        #print("Position",p_tp1_t)
        
        EKF.Mned_tp1_tp1_prev = Mned_tp1_tp1
        EKF.K_tp1_prev = K_tp1
        EKF.P_tp1_t_prev = P_tp1_t
        EKF.n_p_prev = n_p
        EKF.n_z_prev = n_z
        EKF.PHI_prev = PHI
        EKF.delX_tp1_tp1_prev = delX_tp1_tp1
        EKF.v_tp1_tp1_prev = v_tp1_tp1
        EKF.p_tp1_tp1_prev = p_tp1_tp1
        vel.append(np.linalg.norm(v_tp1_tp1))
        pos.append(np.linalg.norm(p_tp1_tp1))
        
        return v_tp1_tp1,p_tp1_tp1



