import time
import serial
from numpy import pi, sin, cos, radians, degrees
from generator import trajectoryGenerator, trajectory, loopTimer
import threading

# Script Config
PLOT_TRAJECTORY = True

SAMPLE_FREQUENCY = 50 # Hz
FILTER_FREQUENCY = 10 # Hz
SERIAL_SPEEDUP = 2 # send serial faster than the sample frequency to ensure buffer filling
REF_SCALE = 10000

XON = b'\x11'
XOFF = b'\x13'

flow_control_paused = False
loop_timing_enabled = False
session_loop_timer = None
session_running = False
recieve_character = None
data_filename = None

ser = serial.Serial(
    port='COM6',
    baudrate=115200
)
generator = trajectoryGenerator(SAMPLE_FREQUENCY, FILTER_FREQUENCY)


def trajectory_command(y,p,r,dy,dp,dr):
    # convert to milliradians(/s)
    m_y = int(y*REF_SCALE)
    m_p = int(p*REF_SCALE)
    m_r = int(r*REF_SCALE)
    m_dy = int(dy*REF_SCALE)
    m_dp = int(dp*REF_SCALE)
    m_dr = int(dr*REF_SCALE)
    return "MY{}P{}R{}y{}p{}r{}\n".format(m_y,m_p,m_r,m_dy,m_dp,m_dr)

def trajectory_command_q(w,x,y,z,dx,dy,dz):
    # convert to milliradians(/s)
    m_w = int(w*REF_SCALE)
    m_x = int(x*REF_SCALE)
    m_y = int(y*REF_SCALE)
    m_z = int(z*REF_SCALE)
    m_dx = int(dx*REF_SCALE)
    m_dy = int(dy*REF_SCALE)
    m_dz = int(dz*REF_SCALE)
    return "QW{}X{}Y{}Z{}x{}y{}z{}\n".format(m_w,m_x,m_y,m_z,m_dx,m_dy,m_dz)

def position_command(y,p,r):
    # convert to milliradians(/s)
    m_y = int(y*1000)
    m_p = int(p*1000)
    m_r = int(r*1000)
    return "MY{}P{}R{}\n".format(m_y,m_p,m_r)

def position_command_q(w,x,y,z):
    # convert to milliradians(/s)
    m_w = int(w*1000)
    m_x = int(x*1000)
    m_y = int(y*1000)
    m_z = int(z*1000)
    return "QW{}X{}Y{}Z{}\n".format(m_w,m_x,m_y,m_z)

def send_command(command):
    ser.write(command.encode("ascii"))

def set_state_open_trajectory():
    send_command("O\n")

def set_state_closed_trajectory():
    send_command("C\n")

def set_state_idle():
    send_command("I\n")

def set_state_position():
    send_command("P\n")

def home_device():
    send_command("H\n")

def enable_motors():
    send_command("E\n")

def disable_motors():
    send_command("D\n")
    
def session_timer_callback(traj: trajectory):
    global session_running, session_loop_timer
    try:
        i = next(traj)
        q = i.q
        omega = i.omega
        command = trajectory_command_q(q[3],q[0],q[1],q[2],omega[0],omega[1],omega[2])
        #print(command)
        send_command(command)
    except StopIteration:
        session_running = False
        session_loop_timer.stop()
        for i in range(5):
            set_state_idle()
            disable_motors()

def remove_byte_at_index(data: bytes, index: int) -> bytes:
    b = bytearray(data)
    del b[index]
    return bytes(b)

def read_with_flow_control():
    global recieve_character
    global session_running
    while True:
        data = ser.read_all()
        if data == None:
            return None
        else:
            if XOFF[0] in data:
                if session_running:
                    session_loop_timer.pause()
                    idx = data.find(XOFF[0])
                    data = remove_byte_at_index(data, idx)
                    #print("flow paused")
            if XON[0] in data:
                if session_running:
                    session_loop_timer.resume()
                    idx = data.find(XON[0])
                    data = remove_byte_at_index(data, idx)
                    #print("flow resumed")
            if b'#' in data:
                idx = data.find(b'#')
                recieve_character = chr(data[idx+1])
                data = remove_byte_at_index(data, idx+1) # remove single-character code sent from device
                data = remove_byte_at_index(data, idx)
                #print("Character Recieved:", recieve_character)
            
            print(data.decode(errors="ignore"), end="")

def wait_for_recieved_character(char):
    global recieve_character
    while (recieve_character != char[0]):
        pass
    recieve_character = None

def trajectory_session(traj: trajectory, closed_loop=True):
    global session_running
    global session_loop_timer
    enable_motors()
    time.sleep(1) # wait for motors to enable
    home_device()
    wait_for_recieved_character('F')
    # Set to Initial position
    set_state_position()
    pos = position_command_q(traj.q[0][3], traj.q[0][0], traj.q[0][1], traj.q[0][2])
    #print(pos)
    send_command(pos)
    # print("cmd:", pos)
    wait_for_recieved_character('F')
    time.sleep(1) # wait a second before sending the trajectory
    # Send trajectory
    session_loop_timer = loopTimer(1/(SAMPLE_FREQUENCY*SERIAL_SPEEDUP), lambda: session_timer_callback(traj))
    
    if closed_loop:
        set_state_closed_trajectory()
    else:
        set_state_open_trajectory()

    session_loop_timer.start()
    session_running = True

def main():
    global session_running
    global session_loop_timer
    print("Trajectory Generator for Coaxial Mainipulator")
    printer_thread = threading.Thread(target=read_with_flow_control, daemon=True)
    printer_thread.start()
    closed_loop = True
    # main loop
    while(True):
        # Client Commands
        line = None
        if not session_running:
            line = input("")

        if line != None:
            if line.startswith("#"): # use has character "#" to bypass commands directly to the microcontroller
                command = line[1:]
                print(command)
                send_command(command)
            else:
                if (session_running == True):
                    print("Cannot start a new session when one is alrady running!")
                else:
                    duration = 20
                # commands to run scripts
                    yf = pf = rf = None
                    if line == "O":
                        closed_loop = False
                        print("Open-Loop Trajectory Control")
                    elif line == "C":
                        closed_loop = True
                        print("Closed-Loop Trajectory Control")
                    elif line == "YT": # yaw triangle
                        yf = generator.triangle_func(radians(45),4)
                        pf = generator.zero_func()
                        rf = generator.zero_func()
                    elif line == "YPT": # yaw-pitch synchronous triangle
                        yf = generator.triangle_func(radians(45),4)
                        pf = generator.triangle_func(radians(30),4)
                        rf = generator.zero_func()
                    elif line == "YRT": # yaw-roll synchronous triangle
                        yf = generator.triangle_func(radians(45),4)
                        pf = generator.zero_func()
                        rf = generator.triangle_func(radians(-30),4)
                    elif line == "PRT": # pitch_roll synchronous triangle
                        yf = generator.zero_func()
                        pf = generator.triangle_func(radians(30),4)
                        rf = generator.triangle_func(radians(30),4)
                    elif line == "YPRT": # yaw-pitch-roll synchronous triangle
                        yf = generator.triangle_func(radians(4.5),0.4)
                        pf = generator.triangle_func(radians(3),0.4)
                        rf = generator.triangle_func(radians(3),0.4)
                    elif line == "YS": # yaw sinusoid
                        yf = generator.sin_func(radians(45),4)
                        pf = generator.zero_func()
                        rf = generator.zero_func()
                    elif line == "YPS": # yaw-pitch off-sync sinusoid
                        yf = generator.sin_func(radians(45),4)
                        pf = generator.cos_func(radians(30),4)
                        rf = generator.zero_func()
                    elif line == "YRS": # yaw-roll off-sync sinusoid
                        yf = generator.sin_func(radians(45),4)
                        pf = generator.zero_func()
                        rf = generator.cos_func(radians(30),4)
                    elif line == "PRS": # yaw-pitch off-sync sinusoid
                        yf = generator.zero_func()
                        pf = generator.sin_func(radians(20),4)
                        rf = generator.cos_func(radians(20),4)
                    
                    # Generate and run trajectory
                    if yf != None:
                        traj = generator.trajectory_from_ypr_func(duration, yf, pf, rf)
                        traj.convert_ypr_to_q()
                        traj.derive_xyz()
                        traj.plot_ypr()
                        traj.plot_q()
                        traj.plot_xyz()
                        trajectory_session((traj), closed_loop=closed_loop)
    
        time.sleep(0.01)

main()