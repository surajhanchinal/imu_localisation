import numpy as np
import math
def quat(roll,pitch,yaw):
    cy = math.cos((yaw * 0.5)*(3.14/180))
    sy = math.sin((yaw * 0.5)*(3.14/180))
    cr = math.cos((roll * 0.5)*(3.14/180))
    sr = math.sin((roll * 0.5)*(3.14/180))
    cp = math.cos((pitch * 0.5)*(3.14/180))
    sp = math.sin((pitch * 0.5)*(3.14/180))


    q0 = cy * cr * cp + sy * sr * sp
    q1 =  cy * sr * cp - sy * cr * sp
    q2 = cy * cr * sp + sy * sr * cp
    q3 = sy * cr * cp - cy * sr * sp
    q = np.array([q0,q1,q2,q3])
    return q

def quat_mul(Q1,Q2):
    #Quaternion multiplication is defined by:
    #(Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
    #(Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
    #(Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
    #(Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2)

    result = np.zeros(4)
    result[0] = Q1[0]*Q2[0] - Q1[1]*Q2[1] - Q1[2]*Q2[2] - Q1[3]*Q2[3]
    result[1] = Q1[0]*Q2[1] + Q1[1]*Q2[0] + Q1[2]*Q2[3] - Q1[3]*Q2[2]
    result[2] = Q1[0]*Q2[2] - Q1[1]*Q2[3] + Q1[2]*Q2[0] + Q1[3]*Q2[1]
    result[3] = Q1[0]*Q2[3] + Q1[1]*Q2[2] - Q1[2]*Q2[1] + Q1[3]*Q2[0]
    return result

def rotate(quaternion,vector):
    gravity = np.zeros(3)
    gravity[0] = 2*( quaternion[1]*quaternion[3] -quaternion[0]*quaternion[2])
    gravity[1] = 2*(quaternion[0]*quaternion[1] + quaternion[2]*quaternion[3])
    gravity[2] = quaternion[0]*quaternion[0] - quaternion[1]*quaternion[1] - quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3]
    gravity = np.multiply(9.8,gravity)
    vector_nog = vector - gravity
    quaternion_tp = np.array([quaternion[0],0,0,0]) + np.multiply(-1,np.insert(quaternion[1:],0,0))
    vector_temp = np.insert(vector_nog,0,0)
    temp = quat_mul(quaternion,vector_temp)
    output = quat_mul(temp,quaternion_tp)

    return output[1:],gravity
