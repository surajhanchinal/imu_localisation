# This python script listens on UDP port 3333
# for messages from the ESP32 board and prints them
import socket
import sys
import time
import csv
import math
#import xlwt
import ZVU
import GPD
import HDR
import AKF as EKF
import quat
import numpy as np
from drawnow import *
import matplotlib.pyplot as plt #import matplotlib library
temp_position = np.array([0,0,0])
vel = [[],[],[]]
pos = [[],[],[]]
flag2 = 1
acc_plt = [[],[],[]]
def makeFig(): #Create a function that makes our desired plot
    plt.figure(1)
    #plt.ylim(0,200)                                 #Set y min and max values
    plt.title('1')      #Plot the title
    plt.grid(True)                                 #Turn the grid on
    plt.ylabel('Temp F')                            #Set ylabels
    plt.plot(vel[0],'r' ,label='Degrees F')       #plot the temperature
    plt.legend(loc='upper left')                    #plot the legend
    plt2=plt.twinx()                                #Create a second y axis
    #plt.ylim(0,200)                           #Set limits of second y axis- adjust to readings you are getting
    plt2.plot(pos[0], 'b', label='Pressure (Pa)') #plot pressure data
    plt2.set_ylabel('Pressrue (Pa)')                    #label second y axis
    plt2.ticklabel_format(useOffset=False)           #Force matplotlib to NOT autoscale y axis
    plt2.legend(loc='upper right')



    plt.figure(2)
    #plt.ylim(0,200)                                 #Set y min and max values
    plt.title('2')      #Plot the title
    plt.grid(True)                                 #Turn the grid on
    plt.ylabel('Temp F')                            #Set ylabels
    plt.plot(vel[1], 'r', label='Degrees F')       #plot the temperature
    plt.legend(loc='upper left')                    #plot the legend
    plt2=plt.twinx()                                #Create a second y axis
    #plt.ylim(0,200)                           #Set limits of second y axis- adjust to readings you are getting
    plt2.plot(pos[1], 'b', label='Pressure (Pa)') #plot pressure data
    plt2.set_ylabel('Pressrue (Pa)')                    #label second y axis
    plt2.ticklabel_format(useOffset=False)           #Force matplotlib to NOT autoscale y axis
    plt2.legend(loc='upper right')



    plt.figure(3)
    #plt.ylim(0,200)                                 #Set y min and max values
    plt.title('3')      #Plot the title
    plt.grid(True)                                 #Turn the grid on
    plt.ylabel('Temp F')                            #Set ylabels
    plt.plot(vel[2], 'r', label='Degrees F')       #plot the temperature
    plt.legend(loc='upper left')                    #plot the legend
    plt2=plt.twinx()                                #Create a second y axis
    #plt.ylim(0,200)                           #Set limits of second y axis- adjust to readings you are getting
    plt2.plot(pos[2], 'b', label='Pressure (Pa)') #plot pressure data
    plt2.set_ylabel('Pressrue (Pa)')                    #label second y axis
    plt2.ticklabel_format(useOffset=False)           #Force matplotlib to NOT autoscale y axis
    plt2.legend(loc='upper right')
    plt.show()

                      #plot the legend                    #plot the legend
gpd = GPD.GPD()
prev = 0
ans = 1
steps = 0
flag = 1
cunt = 0
del_t = 1.0/50.0
zvu_ans = np.array([0,0,0])
num = 1
HDR_obj = HDR.HDR()
ekf = EKF.EKF()
m_tp1 = np.zeros(7)
ZVU_obj = ZVU.ZVU()
velocity = np.array([0,0,0])
position = np.array([0,0,0])
acc_mean = 0.0
f=0
try :
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
except socket.error, msg :
    print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()

try:
    s.bind(('192.168.43.182', 8000))
except socket.error , msg:
    print 'Bind failed. Error: ' + str(msg[0]) + ': ' + msg[1]
    sys.exit()

print 'Server listening'
count = 0
i = 0
time1 = time.time()
time3 = 0
steps = 0
with open('eggs.csv', 'wb') as csvfile:
    fieldnames = ['time']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    while (time.time() < 15 + time1):
        d = s.recvfrom(1024)
        data = d[0]

        if(count == 0):
            time1 = time.time()
            print("inside")
            count=count+1
        i=i+1

        hash1,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,q0,q1,q2,q3 = data.split()
        acceleration = np.array([float(acc_x),float(acc_y),float(acc_z)])

        acceleration = np.multiply(9.8,acceleration)

        angular = np.array([float(gyro_x),float(gyro_y),float(gyro_z)])
        #quater1 = quat.quat(float(roll),float(pitch),float(yaw))
        quater1 = np.array([float(q0),float(q1),float(q2),float(q3)])
        #quaternion_tp = np.array([quater1[0],0,0,0]) + np.multiply(-1,np.insert(quater1[1:],0,0))
        #yaw = math.atan2(2.0 * (quater1[1] * quater1[2] + quater1[0 ] * quater1[3]), quater1[0] * quater1[0] + quater1[1] * quater1[1] - quater1[2] * quater1[2] - quater1[3] * quater1[3])
        yaw = math.atan2(2*quater1[1]*quater1[2] - 2*quater1[0]*quater1[3], 2*quater1[0]*quater1[0] + 2*quater1[1]*quater1[1] - 1)
        pitch =  -1*math.asin(2.0 * (quater1[1] * quater1[3] - quater1[0] * quater1[2]))
        roll  = math.atan2(2.0 * (quater1[0] * quater1[1] + quater1[2] * quater1[3]), quater1[0] * quater1[0] - quater1[1] * quater1[1] - quater1[2] * quater1[2] + quater1[3] * quater1[3])
        #print acceleration - np.array([0,0,9.833])
        yaw = yaw*(180/3.14)
        roll = roll*(180/3.14)
        pitch = pitch*(180/3.14)

        flag = 0
        prev = ans
        acc_t = gpd.update_d(acceleration,angular,quater1)
        ans = gpd.stance_con()
        print(yaw)
        if(ans == 0 and prev == 1 and cunt > 50):##beginning of a step or end of stance phase of previous step
            steps = steps + 1
            gpd.reset()
            print("end of stance",steps)
            cunt = 0
            flag = 1
            num = GPD.GPD.timestep
            #print("time per step ",time.time()-time3,"number of timesteps",num)

            time3 = time.time()
            flag2 = 0
            #velocity = np.array([0,0,0])

        if(ans == 1 and prev == 0):
            print("start of stance")

            #temp_position = position
            velocity = np.array([0,0,0])
            flag2 = 1

        if(flag == 1):
            pass
            #yaw = math.atan2(2.0 * (quater1[1] * quater1[2] + quater1[0] * quater1[3]), quater1[0] * quater1[0] + quater1[1] * quater1[1] - quater1[2] * quater1[2] - quater1[3] * quater1[3])
            #hdr_ans = HDR_obj.Heading(float(yaw))
            #bias_velocity = ZVU_obj.zero_update(velocity,quater1,num*(1.0/94.0))
            #m_tp1[0:1] = hdr_ans
            #m_tp1[1:4] = bias_velocity
            #m_tp1[4:7] = angular
            #print("Bias velocity",bias_velocity)
            #print("heading correction",hdr_ans)

        R = np.diag([0.01,0.01,0.01,0.01,0.5,0.5,0.5])

        #result = ekf.run(angular,acceleration,R,m_tp1,float(roll)*(3.14/180),float(pitch)*(3.14/180),float(yaw)*(3.14/180),flag)
        if(i>250):
            if(f == 0):
                print("start of actual start")
            f = f + 1
            if(not flag2):
                pass
                velocity = velocity + np.multiply(del_t,acc_t)

                position = position + np.multiply(del_t,velocity)
        #print(acc_t)





        vel[0].append(velocity[0])
        vel[1].append(velocity[1])
        vel[2].append(velocity[2])
        acc_plt[0].append(acc_t[0])
        acc_plt[1].append(acc_t[1])
        acc_plt[2].append(acc_t[2])
        pos[0].append(position[0])
        pos[1].append(position[1])
        pos[2].append(position[2])
        #pos[0].append(yaw)
        #pos[1].append(pitch)
        #pos[2].append(roll)
        cunt = cunt + 1


        #print(bias_velocity)
        #x.append(np.linalg.norm(ekf.X_prev))
    s.close()
    time2 = time.time()

    print(i)
    makeFig()
    #plt.plot(GPD.s_c)
    #plt.ylabel('some numbers')
    #plt.show()

    print (time2 - time1,i)
    #print acc_mean/i
"""
count = 0
i = 0
while ((time.time()-time1) < 20):
    data,ADDR = s.recvfrom(1024)
    if(count == 0):
        count=count+1
        time1 = time.time()
    print(data)
    i = i+1
print (time.time() - time1,i)"""
