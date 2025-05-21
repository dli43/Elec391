import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

def animate(i, dataList, ser):
    ser.write(b'g')                                     # Transmit the char 'g' to receive the Arduino data point
    arduinoData_string = ser.readline().decode('ascii').strip() # Decode receive Arduino data as a formatted string


    try:
        ax_str, ay_str = arduinoData_string.split(",")
        ax_val = float(ax_str)
        ay_val = float(ay_str)                          # Convert to float
        theta_rad = math.atan(-ay_val/ax_val)
        theta_deg = math.degrees(theta_rad)
        dataList.append(theta_deg)                          # Add to the list holding the fixed number of points to animate

    except:                                             # Pass if data point is bad                               
        pass

    dataList = dataList[-50:]                           # Fix the list size so that the animation plot 'window' is x number of points
    
    ax.clear()                                          # Clear last data frame
    ax.plot(dataList)                                   # Plot new data frame
    
    ax.set_ylim([-91, 91])                              # Set Y axis limit of plot
    ax.set_title("Arduino Data")                        # Set title of figure
    ax.set_ylabel("Value")                              # Set title of y axis 

dataList = []                                           # Create empty list variable for later use
                                                        
fig = plt.figure()                                      # Create Matplotlib plots fig is the 'higher level' plot window
ax = fig.add_subplot(111)                               # Add subplot to main fig window

ser = serial.Serial("COM4", 9600)                       # Establish Serial object with COM port and BAUD rate to match Arduino Port/rate
time.sleep(2)                                           # Time delay for Arduino Serial initialization 

                                                        # Matplotlib Animation Fuction that takes takes care of real time plot.
                                                        # Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
ani = animation.FuncAnimation(fig, animate, frames=100, fargs=(dataList, ser), interval=100) 

plt.show()                                              # Keep Matplotlib plot persistent on screen until it is closed
ser.close()   