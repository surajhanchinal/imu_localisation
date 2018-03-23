import numpy as np
import math
import quat
#import matplotlib.pyplot as plt #import matplotlib library
#from drawnow import *

#FREQUENCY = 250

#quaternion is 1x4 np array
#acceleration is 3x1 np array same with gyroscope reading

###input read from a UDP server this is where the output comes

s_c = []
s_d = []
s_diff = []
a_plt = []

###end input

class GPD:
    ##define all the variables here
    prev_angular = np.array([0,0,0])
    acc_b_t = []
    prev_local = 0
    prev_acc_m = np.array([0,0,0])
    D_t = [0]
    timestep = 0
    stance_delay_prev = 0
    Mc = 0.8
    Mc_curr = 0
    Md = 0
    Md_curr = 0
    Threshold = 0
    f = 15
    max_temp = 0
    steps = 0
    count = 0
    acc_l = []
    gravity = np.array([0,0,0])
    start = 1
    def update_d(self,curr_acc,angular,quaternion): #### updates the Dynamic Gain

        GPD.timestep = GPD.timestep + 1

        GPD.acc_b_t.append(curr_acc)

        GPD.gravity[0] = 2*( quaternion[1]*quaternion[3] -quaternion[0]*quaternion[2])
        GPD.gravity[1] = 2*(quaternion[0]*quaternion[1] + quaternion[2]*quaternion[3])
        GPD.gravity[2] = quaternion[0]*quaternion[0] - quaternion[1]*quaternion[1] - quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3]
        GPD.gravity = np.multiply(9.8,GPD.gravity)
        acc_no_gravity = curr_acc - GPD.gravity
        #print(acc_no_gravity)
        acc_b = np.insert(acc_no_gravity,0,0)
        acc_ned_temp = quat.quat_mul(quaternion,acc_b)
        quaternion_tp = np.array([quaternion[0],0,0,0]) + np.multiply(-1,np.insert(quaternion[1:],0,0))
        acc_ned = quat.quat_mul(acc_ned_temp,quaternion_tp)
        acc_m = acc_ned[1:]

        if(GPD.timestep<2*GPD.f + 1):
            x = GPD.timestep-1
        else:
            x = 2*GPD.f
            GPD.acc_b_t = GPD.acc_b_t[len(GPD.acc_b_t) - 2*GPD.f:]

        total = 0

        for i in range(x): ### lots of mistakes here
            total_1 = 0
            for j in range(x + 1):
                total_1 = total_1 + GPD.acc_b_t[len(GPD.acc_b_t)-1 - j]

            total_1 = np.multiply(1/(x+1),total_1)

            subtotal = np.linalg.norm(curr_acc - total_1)**(2)  ### this was a mistake i made. i did not square it
            total = total + subtotal

        acc_local = math.sqrt((1/(x+1))*total)

        GPD.D_t.append(np.linalg.norm(acc_local - GPD.prev_local) + abs(np.linalg.norm(acc_m - GPD.prev_acc_m)) + abs(np.linalg.norm(angular - GPD.prev_angular)))

        GPD.prev_local = acc_local
        GPD.prev_acc_m = acc_m
        GPD.prev_angular = angular

        return acc_m



    def stance_con(self): # calculates the stance condition and delay in stance condition





        if(GPD.timestep > 5):
            stance_condition = np.sum(GPD.D_t[len(GPD.D_t)-5:])/5


            GPD.D_t = GPD.D_t[len(GPD.D_t)-5:]

        else:
            stance_condition = np.sum(GPD.D_t)/GPD.timestep


        stance_delay =   (GPD.stance_delay_prev + stance_condition)/2
        if(stance_condition < stance_delay):
            max_t = stance_delay
        else:
            max_t = stance_condition

        #print(stance_condition)
        #drawnow(makeFig)

        GPD.Threshold = GPD.Mc - GPD.Md

        s_c.append(max_t)
        s_d.append(stance_condition - stance_delay)


        if(stance_condition > GPD.Mc_curr):
            GPD.Mc_curr = stance_condition
        if(stance_delay > GPD.Md_curr):
            GPD.Md_curr = stance_delay
        #print(stance_condition,stance_delay,GPD.Threshold)
        GPD.stance_delay_prev = stance_delay
        #print(max_t)
        if( GPD.Threshold >= max_t):
            s_diff.append(-0.5)
            return 1


        else:
            s_diff.append(0.5)
            return 0

    def reset(self):
        #print("new Step for me, new step for me",GPD.Mc_curr -GPD.Md_curr)
        if(GPD.start == 0):
            GPD.Mc = GPD.Mc_curr
            GPD.Md = GPD.Md_curr
            GPD.acc_b_t = []
            GPD.D_t = [0]
            GPD.timestep = 0
            GPD.Mc_curr = 0
            GPD.Md_curr = 0
            GPD.stance_delay_prev = 0
        else:
            GPD.start = 0
        return (GPD.Mc - GPD.Md)
