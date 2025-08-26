import time
import serial
from numpy import pi, sin, cos
from generator import pathGenerator, trajectory, loopTimer
import threading

SAMPLE_FREQUENCY = 20 # Hz
FILTER_FREQUENCY = 10 # Hz
SERIAL_SPEEDUP = 1.5 # send serial faster than the sample frequency to ensure buffer filling

XON = '\x11'
XOFF = '\x13'

flow_control_paused = False
loop_timing_enabled = False
session_loop_timer = None
session_running = False

ser = serial.Serial(
    port='COM6',
    baudrate=115200
)
generator = pathGenerator(SAMPLE_FREQUENCY, FILTER_FREQUENCY)


def movement_command(y,p,r,dy,dp,dr):
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
    command = movement_command(0,0,0,0,0,0)
    send_command(command)

def session_timer_callback(traj_y: trajectory, traj_p: trajectory, traj_r: trajectory):
    try:
        (y, dy, _) = next(traj_y)
        (p, dp, _) = next(traj_p)
        (r, dr, _) = next(traj_r)
        command = movement_command(y,p,r,dy,dp,dr)
        send_command(command)
    except StopIteration:
        session_running = False
        session_loop_timer.stop()
        halt_command()

def read_with_flow_control(serial_object):
    data_buf = []
    data = serial_object.read_all()
    if data == None:
        return None
    else:
        for b in data:
            if b == XOFF:
                session_loop_timer.pause()
                print("flow paused")
            if b == XON:
                session_loop_timer.resume()
                print("flow resumed")
            else:
                data_buf.append(b)
        return bytes(data_buf).decode('ascii')


def main():
    global session_running
    global session_loop_timer
    print("Trajectory Generator for Coaxial Mainipulator")
    halt_command()
    # main loop
    while(True):
        # Application level flow-control
        incoming_data = read_with_flow_control(ser)
        print(incoming_data)
        # Client Commands
        if not session_running:
            line = input("->")
        if line == "T" or line == "t":
            if (session_running == True):
                print("Cannot start a new session when one is alrady running!")
            else:
                session_running = True
                test_duration = 20
                traj_y = generator.generate_zero_trajectory(test_duration)
                traj_p = generator.generate_sin_trajectory(0.75,4,test_duration)
                traj_r = generator.generate_zero_trajectory(test_duration)
                session_loop_timer = loopTimer(1/(SAMPLE_FREQUENCY*SERIAL_SPEEDUP), session_timer_callback, (traj_y, traj_p, traj_r))
                session_loop_timer.start()
                
                # Plot
                if not traj_y.is_zero:
                    traj_y.plot()
                if not traj_p.is_zero:
                    traj_p.plot()
                if not traj_r.is_zero:
                    traj_r.plot()

        if line.startswith("-"): # use dash character '-' to bypass commands directly to the microcontroller
            command = line[1:]
            print(command)
            send_command(command)

main()