#Extended Kalman Filter
import numpy as np
from numpy.linalg import inv

class EKF:
    Mned_tp1_tp1_prev
    P_tp1_t_prev
    n_p_prev
    n_z_prev
    PHI_prev
    delX_tp1_tp1_prev
    v_tp1_tp1_prev
    p_tp1_tp1_prev


    def Extended_Kalman_Filter(deltp1,
                           del_phi,
                           del_w,
                           del_p,
                           del_v,
                           del_ab,
                           ab,
                           flag,
                           del_t,
                           n_p,
                           n_z,
                           alpha_tp1, beta_tp1, Y_tp1):

    #m t+1
        m_tp1=np.array([deltp1, del_w, del_v_i])
    #update equation 34A
        w = w - del_w
    # update acceleration equation 34B
        ab_prime = (ab - del_ab).T

    #calculating Mned equation 35 36 37
   #flag=1 means Stance Phase +
        if flag is 1:
            Mned_tp1_t=np.array([[math.cos(Y_tp1)*math.cos(beta_tp1),(math.cos(Y_tp1)*math.sin(alpha_tp1)*math.sin(beta_tp1))-(math.cos(alpha_tp1)*math.sin(Y_tp1)),(math.sin(alpha_tp1)*math.sin(Y_tp1))+(math.cos(alpha_tp1)*math.cos(Y_tp1)*math.sin(beta_tp1))],
                       [(math.cos(beta_tp1)*math.sin(Y_tp1)),(math.cos(alpha_tp1)*math.cos(Y_tp1))+(math.sin(alpha_tp1)*math.sin(Y_tp1)*math.sin(beta_tp1)),(math.cos(alpha_tp1)*math.sin(Y_tp1)*math.sin(beta_tp1))-(math.cos(Y_tp1)*math.sin(alpha_tp1))],
                       [(-1)*(math.sin(beta_tp1)),math.cos(beta_tp1)*math.sin(alpha_tp1),math.cos(alpha_tp1)*math.cos(beta_tp1)]])
        else:
            del_IpOmegaxdelt=np.array([[2,((-1)*(w[2]))*del_t,w[1]*del_t],
                              [w[2]*del_t,2,((-1)*(w[0]))*del_t],
                              [((-1)*(w[1]))*del_t,w[0]*del_t,2]])
            del_ImOmegaxdelt_inv = inv(del_IpOmegaxdelt.T)
            Mned_tp1_t_term = np.matmul(Mned_tp1_tp1_prev,del_IpOmegaxdelt)
            Mned_tp1_t=np.matmul(Mned_tp1_t_term,del_ImOmegaxdelt_inv)

   #calculating S matrix 32
        
        an_prime=np.matmul(Mned_tp1_t,ab_prime)

        S = np.array([[0,(-1)*(an_prime[2][0]),an_prime[1][0]],[an_prime[2][0],0,(-1)*(an_prime[0][0])],[(-1)*(an_prime[1][0]),an_prime[0][0],0]])


    #calculating  31

        PHI = np.identity(15)
        PHI[0:2,3:5] = np.multiply(del_t,Mned_tp1_t)
        PHI[6:8,9:11] = np.multiply(del_t,np.identity(3))
        PHI[6:8,12:14] = np.multiply(-1*(del_t*del_t)/2,S)
        PHI[9:11,0:2] = np.multiply(-1*del_t,S)
        PHI[9:11,12:14] = np.multiply(del_t,Mned_tp1_t)
        PHI[12:14,12:14] =  np.multiply(-1*del_t,S)

    #delXt+1|t 30
    
        term = np.matmul(PHI,delX_tp1_tp1_prev)
        delX_tp1_t = term + n_p_prev

    #H Matrix 39
        H = np.zeros(7,15)
        H[0:3,2:5] = np.identity(4)
        H[4:6,9:11] = np.identity(3)


    #R_tp1 
        R_tp1 = np.cov(n_z,n_z.T)
    


    #eqn 43
        R_t = np.cov(n_z_prev,n_z_prev.T)
        P_t_t_term1 = np.matmul(K_tp1_prev,R_t)
        P_t_t_term2 = np.matmul(P_t_t_term1,K_tp1_prev.T)
        P_t_t_term3 = np.matmul(K_tp1_prev,H)
        I_15x15 = np.identity(15)
        P_t_t_term4 = I_15x15 - P_t_t_term3
        P_t_t_term5 = np.matmul(P_t_t_term4,P_tp1_t_prev)
        P_t_t_term6 = np.matmul(P_t_t_term5,P_t_t_term4.T)
        P_t_t = P_t_t_term6 + P_t_t_term2


    #eqn 42
        P_tp1_t_term1 = np.matmul(PHI_prev,P_t_t)
        P_tp1_t_term2 = np.matmul(P_tp1_t_term1,PHI_prev.T)
        Q_t = np.cov(n_p_prev,n_p_prev.T)
        P_tp1_t = P_tp1_t_term2 + Q_t



    #eqn 41
        K_tp1_term1 = np.matmul(P_tp1_t,H.T)
        K_tp1_term2 = np.matmul(H,P_tp1_t)
        K_tp1_term3 = np.matmul(K_tp1_term2,H.T)
        K_tp1_term4 = K_tp1_term3 + R_tp1
        K_tp1_term4_inv = inv(K_tp1_term4)
        K_tp1 = np.matmul(K_tp1_term1,K_tp1_term4_inv)

    #eqn 38
        delX_tp1_tp1_term1 = np.matmul(H,delX_tp1_t)
        delX_tp1_tp1_term2 = m_tp1 - delX_tp1_tp1_term1
        delX_tp1_tp1_term3 = np.matmul(K_tp1,delX_tp1_tp1_term2)
        delX_tp1_tp1 = delX_tp1_t + delX_tp1_tp1_term3

    #eqn40
        z_tp1_term1 = np.matmul(H,delX_tp1_tp1)
        z_tp1 = z_tp1_term1 + n_z

    #eqn44
        a_e_tp1_term1 = np.matmul(Mned_tp1_t,ab_prime)
        g_e = np.array(0,0,9.8)
        a_e_tp1 = a_e_tp1_term1 - g_e.T

    #eqn45
        v_tp1_t = v_tp1_tp1_prev + np.multiply(del_t,a_e_tp1)
    #eqn46
        p_tp1_t = p_tp1_tp1_prev + np.multiply(del_t,v_tp1_t)
    
    
    #final update
        v_tp1_tp1 = v_tp1_t - delX_tp1_tp1[9:11]
    
        p_tp1_tp1 = v_tp1_t - delX_tp1_tp1[6:8]

    #Mned t+1|t+1
        del_Imphixdelt=np.array([[2,((-1)*(del_phi[2]))*del_t,del_phi[1]*del_t],
                              [del_phi[2]*del_t,2,((-1)*(del_phi[0]))*del_t],
                              [((-1)*(del_phi[1]))*del_t,del_phi[0]*del_t,2]])
        del_Imphixdelt_inv = inv(del_Imphixdelt)
        Mned_tp1_tp1_term = np.matmul(del_Imphixdelt.T,del_Imphixdelt_inv)
        Mned_tp1_tp1 =np.matmul(Mned_tp1_t_term,Mned_tp1_t)


        Mned_tp1_tp1_prev = Mned_tp1_tp1
        K_tp1_prev = K_tp1
        P_tp1_t_prev = P_tp1_t
        n_p_prev = n_p
        n_z_prev = n_z
        PHI_prev = PHI
        delX_tp1_tp1_prev = delX_tp1_tp1
        v_tp1_tp1_prev = v_tp1_tp1
        p_tp1_tp1_prev = p_tp1_tp1

        return v_tp1_tp1,p_tp1_tp1





    

    

    




