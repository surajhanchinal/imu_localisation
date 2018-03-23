import socket
import sys
def tcp_send(ip_address,port,message):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_address = (ip_address,port)

        sock.connect(server_address)

        try:

            # Send data


            sock.sendall(message)
            print("message sent")

            print('closing socket')
            sock.close()
            return 1
        except:
            print("did not work")
            return 0
