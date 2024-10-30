#This code represents data sent from RasPi module to Output module

import serial
import time
from tkinter import *

# GUI variables
root = Tk()   
root.geometry("600x200")  
valSlider = DoubleVar() 
valOutput = DoubleVar() 
present = BooleanVar()

arduino = serial.Serial(port='COM8',  baudrate=115200, timeout=.1)

def write_read(x): # Initialize Arduino
    arduino.write(bytes(x,  'utf-8'))
    time.sleep(0.05)
    data = arduino.readline()
    return  data

def ArduinoSend(): 
    if present.get():
        valOutput.set(valSlider.get()) 
    else:
        valOutput.set(0)
    print("Current value of v1:", valOutput.get())

    #preprocess data before sending to arduino
    
    value  = write_read(valOutput.get())
    print(value)
    root.after(500, printVal)  # Repeat every 500 ms


#####################################################################################################################\
# Code below is purely for GUI purposes #

def buttPresent():
    present.set(True)

def notPresent():
    present.set(False)


slider = Scale( root, variable = valSlider,  
           from_ = -100, to = 100,  
           orient = HORIZONTAL,
           length=550)    

frame = Frame(root)
detect = Button(frame, text="object present", command=buttPresent)   
empty = Button(frame, text="no object", command=notPresent)   
label = Label(root, text="Slider represents x coordinate of detected object (data from RasPi sent to arduino)") 
label1 = Label(root, text="Button represents whether there is an object (no object detected = raspi has no data to send to arduino)") 
          
detect.pack(side=LEFT, padx=5)
empty.pack(side=LEFT, padx=5)

label.pack(anchor=CENTER, pady=10) 
slider.pack(anchor=CENTER, expand=True)  
label1.pack(anchor=CENTER, pady=10) 
frame.pack(anchor=CENTER, pady=10)


printVal()
root.mainloop()     