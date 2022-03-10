import serial

numbers = """ """
sensoredge1st = 18
sensoredge2nd = 46
a=(64-0)/(sensoredge1st-sensoredge2nd)
b=-(a*sensoredge2nd)
x=18
y=(a*x)+b
print(y)

ser = serial.Serial(
    port='COM4', \
    baudrate=9600, \
    parity=serial.PARITY_NONE, \
    stopbits=serial.STOPBITS_ONE, \
    bytesize=serial.EIGHTBITS, \
    timeout=0.1)

def convertion(angletarget):
    angleadvance = 64-angletarget+18
    teste = (int(angleadvance)).to_bytes(1, byteorder='little')
    print("Valor que será transmitido: ", int.from_bytes(teste, byteorder='little'))
    return angleadvance

def print_menu():
    menu_string = "--- Select an option ---\n"
    menu_string += "1: Min advance\n"
    menu_string += "2: Middle advance\n"
    menu_string += "3: Max advance\n"
    menu_string += "4: Curve advance\n"
    menu_string += "5: Save Calib to Flash\n"
    menu_string += "6: Exit\n"
    print(menu_string)

def open_serial():
    ser.open()

def min_advance():
    global numbers
    
    numbers ="""1
    28
    15000
    1300
    2000
    2500
    3000
    3500
    4000
    4500
    7000
    8000
    9000
    12000
    15000
    64
    64
    64
    64
    64
    64
    64
    64
    64
    64
    64
    64
    90
    80
    10"""    
    preparation()
    
def middle_advance():
    global numbers
    
    numbers ="""1
    28
    15000
    1300
    2000
    2500
    3000
    3500
    4000
    4500
    7000
    8000
    9000
    12000
    15000
    32
    32
    32
    32
    32
    32
    32
    32
    32
    32
    32
    32
    90
    80
    10"""    
    preparation()
    
def max_advance():
    global numbers
    
    numbers ="""1
    28
    15000
    1300
    2000
    2500
    3000
    3500
    4000
    4500
    7000
    8000
    9000
    12000
    15000
    0
    0
    0
    0
    0
    0
    0
    0
    0
    0
    0
    0
    90
    80
    10"""
    preparation()

def curve_advance():
    global numbers
    
    numbers ="""1
    28
    15000
    1300
    2000
    2500
    3000
    3500
    4000
    4500
    7000
    8000
    9000
    12000
    15000
    64
    58
    48
    38
    25
    15
    0
    0
    40
    45
    55
    64
    90
    80
    10"""
    preparation()

def save_curve():
    global numbers
    
    numbers ="""1
    28
    4700
    1500
    2000
    2500
    3000
    3500
    4000
    4500
    5000
    8000
    9000
    12000
    15000
    64
    0
    64
    0
    64
    0
    64
    0
    64
    0
    64
    0
    90
    80
    10"""
    preparation()

def preparation():
    num = str.split(numbers, '\n') 
    result = []
    if selection == 5:
        result.append(0x47) 
    else:
        result.append(0x7E)
    z = (int(num[0])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[1])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))  

    z = (int(num[2])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    z = (int(num[3])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))  
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    z = (int(num[4])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    z = (int(num[5])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    z = (int(num[6])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    z = (int(num[7])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    z = (int(num[8])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    z = (int(num[9])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    z = (int(num[10])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    z = (int(num[11])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    z = (int(num[12])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    z = (int(num[13])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    z = (int(num[14])).to_bytes(2, byteorder='little')
    result.append(int.from_bytes((z[:1]), byteorder='little'))
    result.append(int.from_bytes((z[1:]), byteorder='little'))
    
    z = (int(num[15])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))  
    z = (int(num[16])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[17])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[18])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[19])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[20])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[21])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[22])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[23])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[24])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[25])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[26])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[27])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[28])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    z = (int(num[29])).to_bytes(1, byteorder='little')
    result.append(int.from_bytes(z, byteorder='little'))
    #result.append(0x00)#investigation

    checksum = 0
    checksum = sum(result[:]) & 255

    print('Checksum value:', checksum)
    print('Checksum as bytes:', checksum.to_bytes(1, byteorder='little'))

    z = (int(checksum)).to_bytes(1, byteorder='little')          #investigation
    #result.append(int.from_bytes((z[:1]), byteorder='little'))
    
    result.append(int.from_bytes(z, byteorder='little'))
    
    print(result)    
    frame = bytearray(result)
    print(frame)
    ser.write(frame)
    
def close_serial():
    ser.close() 

if(ser.isOpen() == False):
    open_serial()

selection = 0

try:
    while selection != 6:
        print_menu()
        selection = int(input("Select Option: "))    
        if selection == 1:
            min_advance()
            convertion(18)
            print("Calibration min was sent!!!")
        elif selection == 2:
            middle_advance()
            print("Calibration middle was sent!!!")
        elif selection == 3:
            max_advance()
            print("Calibration max was sent!!!")
        elif selection == 4:
            curve_advance()
            print("Calibration curve was sent!!!")
        elif selection == 5:
            save_curve()
            print("Save Calib to STM32F103 Flash Memory!!!")
            
    print("Exit!!!")
    close_serial()
            
except Exception as err:
    print(f"Invalid input: {err}")    
