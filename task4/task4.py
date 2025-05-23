import time
import serial
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
    
    ax.clear()                                          # Clear last data frame
    ax.plot(accelerometerList)                                  
    ax.plot(gyroscopeList)
    ax.plot(filteredList)
    ax.legend(["accelerometer theta","gyroscope theta","filtered theta"])

    ax.set_ylim([-91, 91])                              # Set Y axis limit of plot
    ax.set_title("Arduino Data")                        # Set title of figure
    ax.set_ylabel("Angle (Degrees)")                              # Set title of y axis 
    ax.set_yticks([-75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75])

gyroscopeList = []              
accelerometerList = []
filteredList = []                                          
                                                        
fig = plt.figure()                                      # Create Matplotlib plots fig is the 'higher level' plot window
ax = fig.add_subplot(111)                               # Add subplot to main fig window

ser = serial.Serial("COM4", 9600)                       # Establish Serial object with COM port and BAUD rate to match Arduino Port/rate
time.sleep(5)                                           # Time delay for Arduino Serial initialization 

                                                        # Matplotlib Animation Fuction that takes takes care of real time plot.
                                                        # Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
ani = animation.FuncAnimation(fig, animate, frames=100, fargs=(gyroscopeList, accelerometerList, filteredList, ser), interval=100) 

plt.show()                                              # Keep Matplotlib plot persistent on screen until it is closed
ser.close()   
