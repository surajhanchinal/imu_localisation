import matplotlib.pyplot as plt

def makeFig(vel,pos): #Create a function that makes our desired plot
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
    plt.plot(vel[2],pos[2], 'r', label='Degrees F')       #plot the temperature
    plt.legend(loc='upper left')                    #plot the legend
    plt.show()
