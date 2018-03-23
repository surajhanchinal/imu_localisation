# This python script listens on UDP port 3333 
# for messages from the ESP32 board and prints them
import socket
import sys
import time
import csv
import xlwt
import GPD
import quat
import numpy as np
from drawnow import *
import matplotlib.pyplot as plt #import matplotlib library

book = xlwt.Workbook(encoding="utf-8")

sheet1 = book.add_sheet("Sheet 1")
def makeFig(): #Create a function that makes our desired plot
    #plt.ylim(0,200)                                 #Set y min and max values
    plt.title('My Live Streaming Sensor Data')      #Plot the title
    plt.grid(True)                                 #Turn the grid on
    plt.ylabel('Temp F')                            #Set ylabels
    plt.plot(GPD.s_c, 'ro-', label='Degrees F')       #plot the temperature
    plt.legend(loc='upper left')                    #plot the legend
    plt2=plt.twinx()                                #Create a second y axis
    #plt.ylim(0,200)                           #Set limits of second y axis- adjust to readings you are getting
    plt2.plot(GPD.s_d, 'b', label='Pressure (Pa)') #plot pressure data
    plt2.set_ylabel('Pressrue (Pa)')                    #label second y axis
    plt2.ticklabel_format(useOffset=False)           #Force matplotlib to NOT autoscale y axis
    plt2.legend(loc='upper right')
    plt.show()
                      #plot the legend                    #plot the legend
gpd = GPD.GPD()
prev = 0
ans = 1
steps = 0
flag = 0
cunt = 0
try :
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
except socket.error, msg :
    print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()

try:
    s.bind(('192.168.43.182', 3333))
except socket.error , msg:
    print 'Bind failed. Error: ' + str(msg[0]) + ': ' + msg[1]
    sys.exit()
     
print 'Server listening'
count = 0
i = 0
#time2 = 0.0
#AHRS =mda.madgwick()
#quat = Quaternion(axis = [1,0,0],angle = 3.14159265)
time1 = time.time()
with open('eggs.csv', 'wb') as csvfile:
    fieldnames = ['time']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()
    while (time.time() < 50 + time1):
        d = s.recvfrom(1024)
        #print(len(d))
        data = d[0]
        
        
        if(count == 0):
            time1 = time.time()
            count=count+1
            
        
        
            
        i=i+1
        
        t,acc_x,acc_y,acc_z,comma_1,mag_x,mag_y,mag_z,comma2,gyro_x,gyro_y,gyro_z,comma_3,yaw,pitch,roll = data.split()
        #print(gyro_y,gyro_x,gyro_z)
        #print(yaw,pitch,roll)
        acceleration = np.array([float(acc_x),float(acc_y),float(acc_z)])
        acceleration = np.multiply(9.8/250.0,acceleration)
        angular = np.array([float(gyro_x),float(gyro_y),float(gyro_z)])
        quater1 = quat.quat(float(roll),float(pitch),float(yaw))
        
        if(flag == 1):
            gpd.update_d(acceleration,angular,quater1,flag)
            
            ans = gpd.stance_con()
            flag = 0
        else:
            gpd.update_d(acceleration,angular,quater1,flag)
            ans = gpd.stance_con( )
        cunt = cunt + 1
        #print ans
        if(ans == 0 and prev == 1 and cunt > 20):
            steps = steps + 1 
            print("new step yoyoyooyoyoyyoyooyoyooyoyoyoyoyoyoyooyoyoyoyoyoyoyoyoyoyooyoyoyyo")
            cunt = 0
            flag = 1
        prev = ans
        
        #writer.writerow({'time':t})
        
        #sheet1.write(i, 0, accx)
        #sheet1.write(i, 1, accy)
        #sheet1.write(i, 2, accz)
        

        #print(data.strip())
    
    s.close()
    book.save("trial.xls")
    makeFig()
    #plt.plot(GPD.s_c)
    #plt.ylabel('some numbers')
    #plt.show()
    
    print (time.time() - time1,i)
"""count = 0
i = 0
while ((time.time()-time1) < 20):
    data,ADDR = s.recvfrom(1024)
    if(count == 0):
        count=count+1
        time1 = time.time()
    print(data) 
    i = i+1
print (time.time() - time1,i)"""
