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
    return com_ports
    
def Connect():
    global s

    ComName = serialchoosen.get()
    
    s = sr.Serial(ComName[0:4],9600)
    
    if s.isOpen():
        print('Connected')
        s.reset_input_buffer()
    else:
        print("Fail")

def Disconnect():

    s.close()

    if s.isOpen():
        print('Fail, still connected...')
    else:
        print("Disconnected")
    
# Plot data
def plot_data():
    global cond, data

    if (s.isOpen()):
        return

    if (cond == True):
        a = s.readline()
        a.decode()

        if(len(data) < 60):
            data = np.append(data,float(a[1:5]))
        else:
            '''
            data[0:99] = data[0:100]
            data[99] = float(a[1:5])
            '''
            data = np.array([])
            data = np.append(data,float(a[1:5]))
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
ax.set_xlim(0,60)
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

root.update()
start = tk.Button(root, text = "Stop ", font = ('calibri',12), command = lambda: plot_stop())
start.place(x = start.winfo_x() + start.winfo_reqwidth() + 450, y = 450)

#Connect
root.update()
start = tk.Button(root, text = "Connect    ", font = ('calibri',12), command = lambda: Connect())
start.place(x = 10, y = 55)

#Disconnect
root.update()
start = tk.Button(root, text = "Disconnect ", font = ('calibri',12), command = lambda: Disconnect())
start.place(x = start.winfo_x() + start.winfo_reqwidth() + 10, y = 55)

# Combobox
# Label
ttk.Label(root, text = "Choose Serial Port:", 
        font = ("Times New Roman", 10)).grid(column = 0, 
        row = 15, padx = 10, pady = 25)
  
n = tk.StringVar()
serialchoosen = ttk.Combobox(root, width = 6, 
                            textvariable = n)

# Adding combobox drop down list
serialchoosen['values'] = serial_available()

serialchoosen.grid(column = 1, row = 15)
  
# Shows COM0 as a default value
serialchoosen.current(0) 

# Start serial port

#s = sr.Serial('COM1',9600)

#s = sr.Serial(com_ports.pop(), 9600)
#s.reset_input_buffer()

root.after(1,plot_data)


root.mainloop()



