import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

def animate(i, dataList, ser, deltaT):
    global reference_angle_computed
    
    if not reference_angle_computed:
      
      ser.write(b'a')                                     # Transmit the char 'a' to receive the starting accelerometer values
      arduinoData_string = ser.readline().decode('ascii').strip() # Decode receive Arduino data as a formatted string

      try:
          ax_str, ay_str, az_str = arduinoData_string.split(",")
          ax_val = float(ax_str)
          ay_val = float(ay_str)
          az_val = float(az_str)                        
          theta_rad = math.atan(ay_val/az_val)
          theta_deg = math.degrees(theta_rad)
          dataList.append(theta_deg)
          reference_angle_computed = True                         

      except:                                             # Pass if data point is bad                               
          pass
      
    else:
        
        ser.write(b'g')
        arduinoData_string = ser.readline().decode('ascii')

        try:
            gx_val= -float(arduinoData_string)
            theta_prev = dataList[-1]
            theta_next = theta_prev + gx_val*deltaT
            dataList.append(theta_next)
        except:
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
reference_angle_computed = False
deltaT = 0.1

                                                        # Matplotlib Animation Fuction that takes takes care of real time plot.
                                                        # Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
ani = animation.FuncAnimation(fig, animate, frames=100, fargs=(dataList, ser, deltaT), interval=deltaT*1e3)

plt.show()                                              # Keep Matplotlib plot persistent on screen until it is closed
ser.close()   
