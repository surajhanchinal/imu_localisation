import matplotlib.pyplot as plt #import matplotlib library
from drawnow import *
import sys
s_c = []
def makeFig(): #Create a function that makes our desired plot
    plt.ylim(0,1000)                                 #Set y min and max values
    plt.title('stance condition')      #Plot the title
    plt.grid(True)                                  #Turn the grid on
    plt.ylabel('stance condition')                            #Set ylabels
    plt.plot(s_c, 'ro-', label='bla bla')       #plot the temperature
    plt.legend(loc='upper left')                    #plot the legend



while 1:
    
    try:
        data = sys.stdin.readline()
        s_c.append(float(data))
        print(data)
        drawnow(makeFig)
    except:
        pass