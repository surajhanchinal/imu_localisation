import udp_class
import time
import numpy as np
import math
import GPD
import quat
from plot import makeFig
from tcp_func import tcp_send
####make objects
udp = udp_class.UDP_data()
gpd = GPD.GPD()
###make objects complete
i = 0
lastUpdate = 0
Now = 0
udp.start("192.168.43.182",3333)
time1 = time.time() 
prev = 0
ans = 1
steps = 0
cunt = 0
acc_plt = [[],[],[]]
vel = [[],[],[]]
pos = [[],[],[]]
del_t = 1/100.0
velocity = np.array([0,0,0])
position = np.array([0,0,0])
time_sample = time.time()
j = 0
flag3  = 0
yaw_offset = 0
while(time.time() - time1 < 40):

    i = i + 1
    data = udp.update()
    #hash1,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,yaw,pitch,roll = data.split()
    hash1,yaw,pitch,roll,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z = data.split()
    acceleration = np.array([float(acc_x),float(acc_y),float(acc_z)])
    angular = np.array([float(gyro_x),float(gyro_y),float(gyro_z)])
    """yaw = float(yaw)
    yaw1 = float(yaw1)
    yaw_offset = 0
    yaw_temp = yaw + yaw_offset
    if(yaw_temp > 180):
        yaw_temp = yaw_temp - 180
    if(yaw_temp < -180):
        yaw_temp = yaw_temp + 180"""
    quater1 = quat.quat(float(roll),float(pitch),float(yaw))
    acc_t = gpd.update_d(acceleration,angular,quater1)
    ans = gpd.stance_con()
    #print(yaw)
    ### if stance condition is ended then set velocity to zero ### This is ZERO VELOCITY UPDATE
    if(prev == 1 and ans == 0 and cunt > 60):#20
        cunt = 0
        steps = steps + 1
        Threshold = gpd.reset()
        print("end of stance : step number : Threshold",steps,Threshold)
        flag3 = 0

        velocity=np.zeros(3)

    if(prev == 0 and ans == 1):
        print("start of stance")
        #tcp_send('192.168.43.205',7800,"S%s,%s:1"%(position[0],position[1]))
        #velocity=np.zeros(3)
    if(cunt > 40):#40
        print("stop")
        velocity = np.array([0,0,0])
        flag3 = 1
    ### update the previous value of stance condition
    prev = ans
    cunt = cunt + 1
    acc_plt[0].append(acc_t[0])
    acc_plt[1].append(acc_t[1])
    acc_plt[2].append(acc_t[2])

    ### end update
    ####UPDATE VELOCITY AND POSITION
    if(time.time()-time_sample < 10):
        j = j + 1
        print(j,yaw)
        #yaw_offset = yaw1 - yaw
    else:
        print("time  :  ",time.time() - time1,yaw)
        #print(yaw)
        del_t = 10.0/j
        if(flag3 == 0):
            velocity = velocity + np.multiply(del_t,acc_t)
            position = position + np.multiply(del_t,velocity)
    vel[0].append(velocity[0])
    vel[1].append(velocity[1])
    vel[2].append(velocity[2])
    pos[0].append(position[0])
    pos[1].append(position[1])
    pos[2].append(position[2])
print(i,(np.linalg.norm(position[:2])*13.1)/2.1,np.linalg.norm(position[:2]))
makeFig([vel[0],vel[1],pos[0]],[pos[0],pos[1],pos[1]])
