import numpy as np

class HDR:
    Yaw_l = 0
    Yaw_2 = 0
    Threshold = 0.5
    def Heading(self,Yaw):
        Psi = Yaw - (HDR.Yaw_l + HDR.Yaw_2)/2

        if Psi > HDR.Threshold:
            Psi = 0
        HDR.Yaw_2 = HDR.Yaw_l
        HDR.Yaw_l = Yaw
        return Psi