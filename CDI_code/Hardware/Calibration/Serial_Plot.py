#Ref code from: https://www.youtube.com/watch?v=0V-6pu1Gyp8

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import tkinter as tk
from tkinter import ttk
import numpy as np
import serial as sr

import serial.tools.list_ports as ports

# Global variables
data = np.array([])
cond = False

# Serial treatment
def serial_available():
    com_ports = list(ports.comports()) # create a list of com ['COM1','COM2'] 
    for i in com_ports:            
        print(i.device) # returns 'COMx' 
    
# Plot data
def plot_data():
    global cond, data

    if (cond == True):
        a = s.redline()
        a.decode()

        if(len(data) < 100):
            data = np.append(data,float(a[0:4]))
        else:
            data[0:99] = data[0:100]
            data[99] = float(a[0:4])
        lines.set_xdata(np.arange(0,len(data)))
        lines.set_ydata(data)

        canvas.draw()

    root.after(1,plot_data)

def plot_start():
    global cond
    cond = True
    s.reset_input_buffer()

def plot_stop():
    global cond
    cond = False

# Main gui code
root = tk.Tk()
root.title('CDI Tool Calibration')
root.configure(background = 'light blue')
root.geometry("900x500") #set the window size

# Create plot object
# add figure canvas
fig = Figure()
ax = fig.add_subplot(111)
ax.set_title('Engine Speed')
ax.set_xlabel('Sample')
ax.set_ylabel('RPM')
ax.set_xlim(0,100)
ax.set_ylim(0,12000)
lines = ax.plot([],[])[0]

canvas = FigureCanvasTkAgg(fig, master=root)
#canvas.get_tk_widget().place(x=10,y=10, width = 500, height = 400)
canvas.get_tk_widget().place(x=350,y=35, width = 500, height = 400)
canvas.draw()

# Create buttons

root.update()
start = tk.Button(root, text = "Start", font = ('calibri',12), command = lambda: plot_start())
#start = tk.Button(root, text = "Start", font = ('calibri',12), command = lambda: serial_available())
start.place(x = 400, y = 450)
print("Start the communication")

root.update()
start = tk.Button(root, text = "Stop ", font = ('calibri',12), command = lambda: plot_stop())
start.place(x = start.winfo_x() + start.winfo_reqwidth() + 450, y = 450)
print("Stop the communication")

# Combobox
# Label
ttk.Label(root, text = "Choose Serial Port:", 
        font = ("Times New Roman", 10)).grid(column = 0, 
        row = 15, padx = 10, pady = 25)
  
n = tk.StringVar()
monthchoosen = ttk.Combobox(root, width = 10, 
                            textvariable = n)
  
# Adding combobox drop down list
monthchoosen['values'] = ('COM0', 
                          'COM1',
                          'COM2',
                          'COM3',
                          'COM4')
  
monthchoosen.grid(column = 1, row = 15)
  
# Shows february as a default value
monthchoosen.current(0) 

# Start serial port

s = sr.Serial('COM4',9600)

#s = sr.Serial(com_ports.pop(), 9600)
s.reset_input_buffer()

root.after(1,plot_data)


root.mainloop()



