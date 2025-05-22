import time
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

def animate(i, ser, deltaT):
    global reference_angle_computed, accelerometerList, gyroscopeList, filteredList
    
    theta_a = get_accelerometer_theta(ser)
    print("theta_a computed")
    if reference_angle_computed:
        print(gyroscopeList[-1])
        theta_g = get_gyroscope_theta(gyroscopeList[-1], deltaT, ser)
        print("theta_g computed")
        theta_k = 0 #get_filtered_theta(theta_a = theta_a, theta_g = get_gyroscope_theta(filteredList[-1], deltaT, ser))
    else:
        theta_g = theta_a
        theta_k = theta_a
        reference_angle_computed = True
    print("all angles computed")
    
    accelerometerList.append(theta_a)
    gyroscopeList.append(theta_g)
    filteredList.append(theta_k)

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
    ax.set_ylabel("Value")                              # Set title of y axis 

def get_accelerometer_theta(ser) -> float:
    ser.write(b'a')
    time.sleep(0.05)                                                 # Transmit the char 'a' to receive the accelerometer values
    arduinoData_string = ser.readline().decode('ascii').strip()     # Decode receive Arduino data as a formatted string

    try:
        ax_str, ay_str, az_str = arduinoData_string.split(",")
        ax_val = float(ax_str)
        ay_val = float(ay_str)
        az_val = float(az_str)                        
        theta_rad = math.atan(ay_val/az_val)
        theta_a = math.degrees(theta_rad)
        return theta_a                     

    except:                                             # Pass if data point is bad                               
        pass

def get_gyroscope_theta(theta_prev: float, deltaT: float, ser) -> float:
    ser.write(b'g')                                          # Transmit the char 'g' to receive the gyroscope values
    print("written successfully")
    time.sleep(0.05)
    arduinoData_string = ser.readline().decode('ascii')         # Decode receive Arduino data as a formatted string
    print("data recieved")
    print(arduinoData_string)
    try:
        g_val= -float(arduinoData_string)
        print("data decoded")
        theta_g = theta_prev + g_val*deltaT
        return theta_g
    except:
        pass
    
def get_filtered_theta(theta_a: float, theta_g: float) -> float:
    k = 0.5
    theta_k = k*theta_g + (1-k) * theta_a
    return theta_k


    
gyroscopeList = []              
accelerometerList = []
filteredList = []
                                                        
fig = plt.figure()                                      # Create Matplotlib plots fig is the 'higher level' plot window
ax = fig.add_subplot(111)                               # Add subplot to main fig window

ser = serial.Serial("COM3", 9600)                       # Establish Serial object with COM port and BAUD rate to match Arduino Port/rate
time.sleep(2)                                           # Time delay for Arduino Serial initialization 
reference_angle_computed = False
deltaT = 0.1

                                                        # Matplotlib Animation Fuction that takes takes care of real time plot.
                                                        # Note that 'fargs' parameter is where we pass in our dataList and Serial object. 
ani = animation.FuncAnimation(fig, animate, frames=100, fargs=(ser, deltaT), interval=deltaT*1e3)

plt.show()                                              # Keep Matplotlib plot persistent on screen until it is closed
ser.close()   
