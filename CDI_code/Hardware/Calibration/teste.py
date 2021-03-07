import serial

frame = bytearray()
frame.append(0x7E) #Command to copy this calibration on RAM
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
frame.append(0x40)
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
frame.append(0x69) #checksum

ser = serial.Serial(
    port='COM3', \
    baudrate=9600, \
    parity=serial.PARITY_NONE, \
    stopbits=serial.STOPBITS_ONE, \
    bytesize=serial.EIGHTBITS, \
    timeout=0.1)


def print_menu():
    menu_string = "--- Select an option ---\n"
    menu_string += "1: Send calibration\n"
    menu_string += "2: xxxx\n"
    menu_string += "3: yyyy\n"
    menu_string += "4: Exit\n"
    print(menu_string)

def open_serial():
    ser.open()

def send_calibration():
    ser.write(frame)
    
def close_serial():
    ser.close() 

if(ser.isOpen() == False):
    open_serial()

selection = 0

try:
    while selection != 4:
        print_menu()
        selection = int(input("Select Option: "))    
        if selection == 1:
            send_calibration()
            print("Calibration was sent!!!")
        elif selection == 2:
            view_friends()
        elif selection == 3:
            remove_friend()
            
    print("Exit!!!")
    close_serial()
            
except Exception as err:
    print(f"Invalid input: {err}")
    
    
"""

void splitByte(unsigned char * split, unsigned int a,int quantBytes)
{
  unsigned char aux;
  int i;
  for(i=0;i<quantBytes;i++)
  {
      split[i]=a&0x00FF; 
      a=(a>>8);
  }
  for(i=0;i<quantBytes-1;i++)
  {
      aux = split[i];
      split[i] = split[quantBytes-i-1];
      split[quantBytes-i-1] = aux;

  }
}

text = hex(7500)
splitByte(text, result, 1)

b'\x31\x01\xDC\x09'

28
7500
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
10

{ 28, 7500,{ 1300, 2000, 2500, 3000, 3500, 4000, 4500, 7000, 8000, 9000,12000,15000},{   64,   58,   48,   38,   25,   15,    0,    0,   40,   45,   55,   64}, 90, 80, 10};
