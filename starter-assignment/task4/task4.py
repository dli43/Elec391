import time
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def animate(i, gyroscopeList, accelerometerList, filteredList, ser):
    ser.write(b'g')
    arduinoData_string = ser.readline().decode('ascii').strip()     # Decode receive Arduino data as a formatted string
    theta_a_str, theta_g_str, theta_k_str = arduinoData_string.split(",")

    try:
        theta_a = float(theta_a_str)
        theta_g = float(theta_g_str)
        theta_k = float(theta_k_str)                       
        accelerometerList.append(theta_a)                   
        gyroscopeList.append(theta_g)
        filteredList.append(theta_k)                                  # Add data points to list

    except:                                             # Pass if data point is bad  
        pass

    accelerometerList = accelerometerList[-50:]   
    gyroscopeList = gyroscopeList[-50:]
    filteredList = filteredList[-50:] 

    yticks = np.linspace(-y_margin,y_margin,resolution)
                     
    ax.clear()                                          
    ax.plot(accelerometerList)  
    ax.set_ylim([-y_margin, y_margin])                                                     
    ax.set_ylabel("Angle (Degrees)")                              
    ax.set_yticks(yticks)
    
    if subplots:
        ax.set_title("Accelerometer Theta")
        
        ay.clear()                                
        ay.plot(gyroscopeList,"g")
        ay.set_ylim([-y_margin, y_margin])                              
        ay.set_title("Gyroscope Theta")                       
        ay.set_ylabel("Angle (Degrees)")                              
        ay.set_yticks(yticks)
        
        az.clear()
        az.plot(filteredList,"r")
        az.set_ylim([-y_margin, y_margin])                              
        az.set_title("Filtered Theta")                       
        az.set_ylabel("Angle (Degrees)")                              
        az.set_yticks(yticks)
    else:
        ax.plot(gyroscopeList)
        ax.plot(filteredList)
        ax.legend(["accelerometer theta","gyroscope theta","filtered theta"])   
    plt.grid()     


gyroscopeList = []              
accelerometerList = []
filteredList = []            

subplots = False
y_margin = 10 
resolution = 9 
                                                        
fig = plt.figure()    
                                  # Create Matplotlib plots fig is the 'higher level' plot window
if subplots:
    ax = fig.add_subplot(311)                               
    ay = fig.add_subplot(312)
    az = fig.add_subplot(313)
else:
    ax = fig.add_subplot(111)

ser = serial.Serial("COM5", 115200)                       # Establish Serial object with COM port and BAUD rate to match Arduino Port/rate
time.sleep(2)                                           # Time delay for Arduino Serial initialization 

                                                        # Matplotlib Animation Fuction that takes takes care of real time plot.
                                                        # Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
ani = animation.FuncAnimation(fig, animate, frames=100, fargs=(gyroscopeList, accelerometerList, filteredList, ser), interval=25) 

plt.show()                                              # Keep Matplotlib plot persistent on screen until it is closed
ser.close()   
