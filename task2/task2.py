import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def animate(i, dataList, ser):
    ser.write(b'g')                                                 # Transmit the char 'g' to receive the Arduino data point
    arduinoData_string = ser.readline().decode('ascii')             # Decode receive Arduino data as a formatted string

    try:
        theta = float(arduinoData_string)                       
        dataList.append(theta)                                  # Add data point to list

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