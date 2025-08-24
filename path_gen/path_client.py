import time
import serial
from numpy import pi, sin, cos


path = ['1', '2', '3', '4', '5', '6', '7', 'A', 'B', 'C']

ser = serial.Serial(
    port='COM6',
    baudrate=115200,
    #parity=serial.PARITY_ODD,
    #stopbits=serial.STOPBITS_TWO,
    #bytesize=serial.SEVENBITS
)

def generate_command(y,p,r,dy,dp,dr):
    # convert to milliradians(/s)
    m_y = int(y*1000)
    m_p = int(p*1000)
    m_r = int(r*1000)
    m_dy = int(dy*1000)
    m_dp = int(dp*1000)
    m_dr = int(dr*1000)
    return "MY{}P{}R{}y{}p{}r{}\n".format(m_y,m_p,m_r,m_dy,m_dp,m_dr)

def send_command(command):
    ser.write(command.encode('ascii'))  ## Encode the string as ASCII and write it to the serial port

def command_halt():
    command = generate_command(0,0,0,0,0,0)
    send_command(command)

def test_serial():
    print("Testing trajecotry generation")
    test_duration = 10 #seconds
    start_time = time.time()
    while time.time() < start_time + test_duration:
        t = time.time()
        dy = 2*sin(t)
        dy = 2*cos(t) # derivative of y

        p = 0.5*sin(t)
        dp = 0.5*cos(t) # derivative of p

        r = 0.5*cos(t)
        dr = 0.5*-sin(t) # derivative of r
        
        command = generate_command(0,p,r,0,dp,dr)
        print(command, end="")
        send_command(command)
        time.sleep(0.05) # 20 updates per second
    command_halt()
    print("finished test")

def main():
    print("Trajectory Generator for Coaxial Mainipulator")
    command_halt()
    # main loop
    while(True):
        line = input("->")
        if line == "T" or line == "t":
            test_serial()
        if line.startswith("-"):
            command = line[1:]
            print(command)
            send_command(command)

main()