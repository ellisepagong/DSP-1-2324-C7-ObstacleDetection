# Code simulates data sent from RasPi module to Output module

import serial
import time
from tkinter import *

# GUI variables
root = Tk()   
root.geometry("600x200")  
valSlider = DoubleVar() 
valOutput = DoubleVar() 
present = BooleanVar()
upper = DoubleVar() # upper boundary of x coordinate

upper = 200
present.set(False)

# arduino = serial.Serial(port='COM8',  baudrate=115200, timeout=.1)

# def write_read(x): # Initialize Arduino
#     arduino.write(bytes(x,  'utf-8'))
#     time.sleep(0.05)


def getBin(x, high_boundary=upper): # encode x coordinate to binary
    segment_size = (high_boundary) // 5
    for i in range(5):
        low = 1 + i * segment_size
        high = low + segment_size - 1
        if i == 5 - 1:
            high = high_boundary
        if low <= x <= high:
            binary_value = format(i + 1, '03b')
            return binary_value

    return format(0, '03b')  # In case x is outside the range


def ardSend(): # send data to arduino
    if present.get(): 
        # Output is x coordinate of object given presence of object
        valOutput.set(valSlider.get()) 
    else:
        #  Output is 0 if there is no object sent
        valOutput.set(0)

    # encode data before sending to arduino  
    out = getBin(int(valOutput.get()))
    # write_read(out) # Send to arduino

    print("Current value: ", valOutput.get()," Current binary: ", out)
    root.after(500, ardSend)  # Repeat every 500 ms


## Code below is for GUI purposes ##

def buttPresent():
    present.set(True)

def notPresent():
    present.set(False)


slider = Scale( root, variable = valSlider, from_ = 1, to = upper, orient = HORIZONTAL, length=550)    

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

ardSend()
root.mainloop()     