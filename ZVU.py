import numpy as np
gravity = np.array([0,0,9.833])
import quat
class ZVU:

    def zero_update(self,velocity,quaternion,time):
            acc_m = np.multiply(1.0/time,velocity)
            #print velocity.shape
            acc_e = acc_m + gravity
            acc_e = np.insert(acc_e,0,0)##dont know why this gives me an error
            #print acc_e.shape
            quaternion_tp = np.array([quaternion[0],0,0,0]) + np.multiply(-1,np.insert(quaternion[1:],0,0))
            #print "hello"
            #print quaternion_tp.shape
            acc_ned_temp = quat.quat_mul(quaternion_tp,acc_e)

            #print temp
            #print temp.shape
            acc_b = quat.quat_mul(acc_ned_temp,quaternion)
            #print acc_b.shape
            #print acc_b
            acc_actual = acc_m - acc_b[1:]
            bias_velocity = np.multiply(time,acc_b[1:])
            return bias_velocity