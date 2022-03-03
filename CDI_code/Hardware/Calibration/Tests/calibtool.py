import serial

frame = bytearray()
frame.append(0x73) #Command to copy this calibration on RAM
frame.append(0x1C)
frame.append(0x00)
frame.append(0x4C)
frame.append(0x1D)
frame.append(0x14)
frame.append(0x05)
frame.append(0xD0)
frame.append(0x07)
frame.append(0xC4)
frame.append(0x09)
frame.append(0xB8)
frame.append(0x0B)
frame.append(0xAC)
frame.append(0x0D)
frame.append(0xA0)
frame.append(0x0F)
frame.append(0x94)
frame.append(0x11)
frame.append(0x58)
frame.append(0x1B)
frame.append(0x40)
frame.append(0x1F)
frame.append(0x28)
frame.append(0x23)
frame.append(0xE0)
frame.append(0x2E)
frame.append(0x98)
frame.append(0x3A)
frame.append(0x30)
frame.append(0x26)
frame.append(0x19)
frame.append(0x0F)
frame.append(0x00)
frame.append(0x00)
frame.append(0x28)
frame.append(0x2D)
frame.append(0x37)
frame.append(0x40)
frame.append(0x5A)
frame.append(0x50)
frame.append(0x0A)
frame.append(0x00)
frame.append(0x2E) #checksum

ser = serial.Serial(
    port='COM1', \
    baudrate=9600, \
    parity=serial.PARITY_NONE, \
    stopbits=serial.STOPBITS_ONE, \
    bytesize=serial.EIGHTBITS, \
    timeout=0.1)


def print_menu():
    menu_string = "--- Select an option ---\n"
    menu_string += "1: Send the calibration"
    menu_string += "2: xxxx"
    menu_string += "3: yyyy"
    menu_string += "4: Exit\n"
    print(menu_string)

def open_serial():
    ser.open()

def send_calibration():
    ser.write(frame)
    
def close_serial():
    ser.close() 

open_serial()

selection = 0

try:
    while selection != 4:
        print_menu()
        selection = int(input("Select Option: "))    
        if selection == 1:
            send_calibration()
        elif selection == 2:
            view_friends()
        elif selection == 3:
            remove_friend()
            
excpet Exception as err:
    print(f"Invalid input: {err}")
       









"""
ang = 28
max_engspeed = 7500

print(ang.to_bytes((1, 'big'))
print(max_engspeed.to_bytes((2, 'big'))

frame = b""
frame += ang.to_bytes((1, 'big')
frame += max_engspeed.to_bytes((2, 'big')
bytesList = list(frame)
print(bytesList)

#int.from_bytes(b'\x00\x10', byteorder='big')
#.to_bytes(10, byteorder='big', signed=True)





ser.is_open()

string = "Hello World"

# string with encoding 'utf-8'
arr = bytes(string, 'utf-8')
arr2 = bytes(string, 'ascii')

print(arr,'\n')

# actual bytes in the the string
for byte in arr:
    print(byte, end=' ')
print("\n")
for byte in arr2:
    print(byte, end=' ')
    
    
frame = b""
frame += ang.to_bytes((1, 'big')
frame += max_engspeed.to_bytes((2, 'big')
bytesList = list(frame)    
    
frame = b""
frame += bytes(int('0xA2', 16).to_bytes(1, "big"))
frame += bytes(int('0x01', 16).to_bytes(1, "big"))
frame += bytes(int('0x02', 16).to_bytes(1, "big"))
frame += bytes(int('0x03', 16).to_bytes(1, "big"))
frame += bytes(int('0x04', 16).to_bytes(1, "big"))
bytesList = list(frame)

>>> frame = bytearray()
>>> frame.append(0xA2)
>>> frame.append(0x01)
>>> frame.append(0x02)
>>> frame.append(0x03)
>>> frame.append(0x04)
>>> frame
bytearray(b'\xa2\x01\x02\x03\x04')
or, using your code but fixing the errors:

frame = b""
frame += b'\xA2' 
frame += b'\x01' 
frame += b'\x02' 
frame += b'\x03'
frame += b'\x04'    
    
    
    
# Cast bytes to bytearray
mutable_bytes = bytearray(b'\x00\x0F')

# Bytearray allows modification
mutable_bytes[0] = 255
mutable_bytes.append(255)
print(mutable_bytes)

# Cast bytearray back to bytes
immutable_bytes = bytes(mutable_bytes)
print(immutable_bytes)    


def int_to_bytes(val, num_bytes):
    return [(val & (0xff << pos*8)) >> pos*8 for pos in range(num_bytes)]



def int_to_bytes(val, num_bytes):
    return [(val & (0xff << pos*8)) >> pos*8 for pos in reversed(range(num_bytes))] 

  
a = int_to_bytes(7500, 2)
print(a[0])

7E 1C 00 4C 1D 14 05 D0 07 C4 09 B8 0B AC 0D A0          0F 94 11 58 1B 40 1F 28 23 E0 2E 98 3A 40 3A 30 26 19 0F 00 00 28 2D 37 40 5A 50 0A 00 2E 00


"""

    
