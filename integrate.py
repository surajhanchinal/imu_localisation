import numpy as np
class int_correct:
    velocity = np.array(0,0,0)
    position = np.array(0,0,0)
    rotation_matrix_prev
    delta = 0
    acc_e = 0
    def __init__(self,frequency):
        int.delta = 1./(frequency) 

    def integ(acceleration,rot_matrix_prev,err_vec):
        
        
        int_correct.acc_e = acceleration - err_vec
        
        if flag is 1
            Mned_tp1_t=np.array([[math.cos(Y_tp1)*math.cos(beta_tp1),(math.cos(Y_tp1)*math.sin(alpha_tp1)*math.sin(beta_tp1))-(math.cos(alpha_tp1)*math.sin(Y_tp1)),(math.sin(alpha_tp1)*math.sin(Y_tp1))+(math.cos(alpha_tp1)*math.cos(Y_tp1)*math.sin(beta_tp1))],
                       [(math.cos(beta_tp1)*math.sin(Y_tp1)),(math.cos(alpha_tp1)*math.cos(Y_tp1))+(math.sin(alpha_tp1)*math.sin(Y_tp1)*math.sin(beta_tp1)),(math.cos(alpha_tp1)*math.sin(Y_tp1)*math.sin(beta_tp1))-(math.cos(Y_tp1)*math.sin(alpha_tp1))],
                       [(-1)*(math.sin(beta_tp1)),math.cos(beta_tp1)*math.sin(alpha_tp1),math.cos(alpha_tp1)*math.cos(beta_tp1)]])
        else
             del_IpOmegaxdelt=np.array([[2,((-1)*(del_w_k))*del_t,del_w_j*del_t],
                              [del_w_k*del_t,2,((-1)*(del_w_i))*del_t],
                              [((-1)*(del_w_j))*del_t,del_w_i*del_t,2]])
            del_ImOmegaxdelt_inv = np.linalg.inv(np.transpose(del_IpOmegaxdelt))
            Mned_tp1_t_term = np.matmul(Mned_tp1_tp1_prev,del_IpOmegaxdelt)
            Mned_tp1_t=np.matmul(Mned_tp1_t_term,del_ImOmegaxdelt_inv)
        
        acc_corr = np.matmul(Mned_tp1_t,int_correct.acc_e)

        int_correct.velocity = int_correct.velocity + np.multiply(int_correct.delta,acc_corr)
        int_correct.position = int_correct.position + np.multiply(int_correct.delta,int_correct.velocity)

        return int_correct.velocity,int_correct.position,int_correct.rotation_matrix_prev
