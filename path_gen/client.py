import time
import serial
from numpy import pi, sin, cos
from generator import pathGenerator

SAMPLE_FREQUENCY = 50 # Hz
FILTER_FREQUENCY = 10 # Hz

flow_control_paused = False
ser = serial.Serial(
    port='COM6',
    baudrate=115200,
    xonxoff = True
)

generator = pathGenerator(SAMPLE_FREQUENCY, FILTER_FREQUENCY)

def generate_movement_command(y,p,r,dy,dp,dr):
    # convert to milliradians(/s)
    m_y = int(y*1000)
    m_p = int(p*1000)
    m_r = int(r*1000)
    m_dy = int(dy*1000)
    m_dp = int(dp*1000)
    m_dr = int(dr*1000)
    return "MY{}P{}R{}y{}p{}r{}\n".format(m_y,m_p,m_r,m_dy,m_dp,m_dr)

def send_command(command):
    ser.write(command.encode("ascii"))

def start_command():
    send_command("S\n")

def stop_command():
    send_command("X\n")

def halt_command():
    command = generate_movement_command(0,0,0,0,0,0)
    send_command(command)

def read_printout():
    while ser.in_waiting:
        line = ser.readline()
        print(str(line))

def main():
    print("Trajectory Generator for Coaxial Mainipulator")
    halt_command()
    # main loop
    while(True):
        line = input("->")
        if line == "T" or line == "t":
            trajectory = generator.generate_triangle_trajectory(1,2,10)
            trajectory.plot()
        if line.startswith("-"): # use dash character '-' to bypass commands directly to the microcontroller
            command = line[1:]
            print(command)
            send_command(command)

main()