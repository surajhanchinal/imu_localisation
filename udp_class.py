import socket
import sys
import time
acc_plt = [[],[],[]]

class UDP_data:
    s  = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    def start(self,ip_address,port):
        try :
            UDP_data.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            UDP_data.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            UDP_data.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1) ## depends
        except socket.error, msg :
            print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
            sys.exit()

        try:
            UDP_data.s.bind((ip_address, port))
        except socket.error , msg:
            print 'Bind failed. Error: ' + str(msg[0]) + ': ' + msg[1]
            sys.exit()

        print 'Server listening'
        return

    def update(self):
            data,ADDR = UDP_data.s.recvfrom(1024)
            count = 0
            if(count == 0):
                #print("started receiving")
                count = count+1
            return data

    def stop(self):
            UDP_data.s.close()
