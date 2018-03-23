import numpy as np
import math
#H = np.zeros((7,15))
##H[0:4,2:6] = np.identity(4)
#H[4:7,9:12] = np.identity(3)
P_t_t = np.identity(15)
P_t_t = np.diag([0.0,0.0,0.0,0.1,0.1,0.1,0,0,0,0,0,0,0.1,0.1,0.1])
vel = [[],[],[]]
pos = [[],[],[]]
acc_gr = [[],[],[]]

vel1 = [[],[],[]]
pos1= [[],[],[]]
acc_gr1 = [[],[],[]]
acc_no = [[],[],[]]
class EKF:

    del_t = 1.0/49.0
    M_NED_i = np.zeros((3,3))
    PHI = np.zeros((15,15))
    M_NED_prev = np.zeros((3,3))
    Kalman_prev = np.zeros((15,7))
    P_prev = np.empty_like(P_t_t)
    P_prev[:] = P_t_t
    P_i = np.empty_like(P_t_t)
    Q = np.diag([0.0001 , 0.0001 ,0.0001 , 0, 0, 0, 0, 0, 0, 0.0001,0.0001 , 0.0001, 0, 0, 0])
    gravity = np.array([0,0,9.8])
    R_prev = np.diag([0.01,0.01,0.01,0.01,0.01,0.01,0.01])
    X_prev = np.zeros((15,1))
    #X_prev[0:3,0] = np.array([0.1,0.1,0.1])
    #X_prev[12:15,0] = np.array([0.01,0.01,0.89])
    velocity_prev = np.array([0,0,0])
    position_prev = np.array([0,0,0])
    process_noise = np.zeros(15)
    cunt = 1
    H = np.zeros((7,15))
    H[0:4,2:6] = np.identity(4)
    H[4:7,9:12] = np.identity(3)
    acc_mean = 0.0



    def state_transition_matrix(self,ang,acc,flag):
        ang_cor = ang - EKF.X_prev[3:6,0]
        acc_cor = acc - EKF.X_prev[12:15,0] - EKF.gravity

        Omega = np.array([[0,-1*ang_cor[2],ang_cor[1]],
                          [ang_cor[2],0,-1*ang_cor[0]],
                          [-1*ang_cor[1],ang_cor[0],0]])

        numerator = np.multiply(2,np.identity(3)) + np.multiply(EKF.del_t,Omega)

        denominator = np.multiply(2,np.identity(3)) - np.multiply(EKF.del_t,Omega)
        if(0):
            EKF.M_NED_i = np.matmul(EKF.M_NED_prev,np.matmul(numerator,np.linalg.inv(denominator)))

        acc_cor_n = np.matmul(EKF.M_NED_i,acc_cor)

        S = np.array([[0,-1*acc_cor_n[2],acc_cor_n[1]],
                          [acc_cor_n[2],0,-1*acc_cor_n[0]],
                          [-1*acc_cor_n[1],acc_cor_n[0],0]])

        EKF.PHI = np.identity(15)
        EKF.PHI[0:3,3:6] = np.multiply(EKF.del_t,EKF.M_NED_i)
        EKF.PHI[6:9,9:12] = np.multiply(EKF.del_t,np.identity(3))
        EKF.PHI[6:9,12:15] = np.multiply(-1*(EKF.del_t*EKF.del_t)/2,S)
        EKF.PHI[9:12,0:3] = np.multiply(-1*EKF.del_t,S)
        EKF.PHI[9:12,12:15] = np.multiply(EKF.del_t,EKF.M_NED_i)
        EKF.PHI[12:15,12:15] =  np.multiply(-1*EKF.del_t,S)

        """Omega = np.array([[0,-1*EKF.X_prev[0,0],EKF.X_prev[1,0]],
                          [EKF.X_prev[2,0],0,-1*EKF.X_prev[0,0]],
                          [-1*EKF.X_prev[1,0],EKF.X_prev[0,0],0]])

        numerator = np.multiply(2,np.identity(3)) + np.multiply(EKF.del_t,Omega)

        denominator = np.multiply(2,np.identity(3)) - np.multiply(EKF.del_t,Omega)

        EKF.M_NED_prev = np.matmul(EKF.M_NED_i,np.matmul(numerator,np.linalg.inv(denominator)))"""


        return acc_cor_n,acc_cor


    def error_covariance_matrix(self):
        print np.trace(EKF.P_prev)
        EKF.P_i = np.matmul(np.matmul(EKF.PHI,EKF.P_prev),np.transpose(EKF.PHI)) + EKF.Q
        #print("P_i ka shape",EKF.P_i.shape)
        temp = np.identity(15) - np.matmul(EKF.Kalman_prev,EKF.H)
        #print("temp in ecm  shape",temp.shape)

        EKF.P_prev =  np.matmul(np.matmul(temp,EKF.P_i),np.transpose(temp)) + np.matmul(np.matmul(EKF.Kalman_prev,EKF.R_prev),np.transpose(EKF.Kalman_prev))
        #print("naya P_prev ka shape",P_prev.shape)

    def Kalman_gain(self,R):

        temp = np.linalg.inv(np.matmul(np.matmul(EKF.H,EKF.P_i),np.transpose(EKF.H))+R)
        #print("kalman gain me temp ka shape",temp.shape)
        EKF.Kalman_prev = np.matmul(np.matmul(EKF.P_i,np.transpose(EKF.H)),temp)
        #print("kalman_prev ka shape",EKF.Kalman_prev.shape)


    def update_X(self,m):
        #print("print",np.matmul(EKF.PHI,EKF.X_prev).shape)
        temp2 =  np.matmul(EKF.PHI,EKF.X_prev) + EKF.process_noise[:,None]
        X_i = temp2
        #print("X_i ka shape",X_i.shape)
        temp = m[:,None] - np.matmul(EKF.H,X_i)
        #print("temp in update_x",temp.shape)
        temp3 = np.matmul(EKF.Kalman_prev,temp)
        #print("temp3 in update_x",temp3.shape)
        EKF.X_prev = X_i + temp3
        #print("X_prev in update_x")


    def calculate_M_ned(self,roll,pitch,yaw,flag):

        if(flag):
            cr = math.cos(roll*(3.14/180))
            sr = math.sin(roll*(3.14/180))
            cp = math.cos(pitch*(3.14/180))
            sp = math.sin(pitch*(3.14/180))
            cy = math.cos(yaw*(3.14/180))
            sy = math.sin(yaw*(3.14/180))
            EKF.M_NED_i = np.array([[cy*cp,cy*sr*sp - cr*sy,sr*sy + cr*cy*sp],
            [cp*sy,cr*cy + sr*sy*sp,cp*sy*sp - cy*sr],
            [-1*sp,cp*sr,cr*sp]])




    def run(self,ang,acc,R,m,roll,pitch,yaw,flag):

        self.calculate_M_ned(roll,pitch,yaw,1)
        ##lots of other things but for now
        self.error_covariance_matrix()
        acc_t,acc_non = self.state_transition_matrix(ang,acc,flag)
        if(flag == 1):
            EKF.P_prev = P_t_t

        self.Kalman_gain(R)
        self.update_X(m)
        #acc_e = acc
        #print EKF.velocity_prev.shape

        EKF.velocity_prev = EKF.velocity_prev + np.multiply(EKF.del_t,acc_t)
        EKF.position_prev = EKF.position_prev + np.multiply(EKF.del_t,acc_t)

        EKF.velocity_prev = EKF.velocity_prev - EKF.X_prev[9:12,0]

        EKF.position_prev = EKF.position_prev - EKF.X_prev[6:9,0]

        EKF.acc_mean = EKF.acc_mean + acc_t

        #print EKF.P_i.shape
        vel[0].append(EKF.velocity_prev[0])
        acc_gr[0].append(acc_t[0])
        vel[1].append(EKF.velocity_prev[1])
        acc_gr[1].append(acc_t[1])
        vel[2].append(EKF.velocity_prev[2])
        acc_gr[2].append(acc_t[2])

        pos[0].append(EKF.position_prev[0])
        pos[1].append(EKF.position_prev[1])
        pos[2].append(EKF.position_prev[2])

        acc_no[0].append(acc_non[0])
        acc_no[1].append(acc_non[1])
        acc_no[2].append(acc_non[2])
        EKF.R_prev = R

        return acc_t,EKF.position_prev
