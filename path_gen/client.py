import time
import serial
from numpy import pi, sin, cos, radians, degrees
from generator import pathGenerator, trajectory, loopTimer
import threading

SAMPLE_FREQUENCY = 50 # Hz
FILTER_FREQUENCY = 10 # Hz
SERIAL_SPEEDUP = 2 # send serial faster than the sample frequency to ensure buffer filling

XON = b'\x11'
XOFF = b'\x13'

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

def open_trajectory_command():
    send_command("O\n")

def stop_command():
    send_command("X\n")

def halt_command():
    command = movement_command(0,0,0,0,0,0)
    send_command(command)

def session_timer_callback(traj_y: trajectory, traj_p: trajectory, traj_r: trajectory):
    global session_running, session_loop_timer
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
        stop_command()

def read_with_flow_control(serial_object):
    data = serial_object.read_all()
    if data == None:
        return None
    
    data_buf = []
    for b in data:
        if b == XOFF[0]:
            session_loop_timer.pause()
            #print("flow paused")
        elif b == XON[0]:
            session_loop_timer.resume()
            #print("flow resumed")
        else:
            data_buf.append(b)
    
    if data_buf:
        return bytes(data_buf).decode(errors="ignore")
    return None

def main():
    global session_running
    global session_loop_timer
    print("Trajectory Generator for Coaxial Mainipulator")
    halt_command()
    # main loop
    while(True):
        incoming_data = read_with_flow_control(ser)
        if incoming_data != None:
            print(incoming_data, end="")
        # Client Commands
        if not session_running:
            line = input("->")
        if line == "T" or line == "t":
            line = None
            if (session_running == True):
                print("Cannot start a new session when one is alrady running!")
            else:
                session_running = True
                test_duration = 20
                traj_y = generator.generate_sin_trajectory(radians(40),4,test_duration, filter=True)
                traj_p = generator.generate_sin_trajectory(radians(40),4,test_duration, filter=True)
                traj_r = generator.generate_zero_trajectory(test_duration)
                session_loop_timer = loopTimer(1/(SAMPLE_FREQUENCY*SERIAL_SPEEDUP), session_timer_callback, (traj_y, traj_p, traj_r))
                open_trajectory_command()
                session_loop_timer.start()
                
                # Plot
                if not traj_y.is_zero:
                    traj_y.plot()
                if not traj_p.is_zero:
                    traj_p.plot()
                if not traj_r.is_zero:
                    traj_r.plot()
        
        time.sleep(0.01)

        if line != None:
            if line.startswith("-"): # use dash character '-' to bypass commands directly to the microcontroller
                command = line[1:]
                print(command)
                send_command(command)

main()