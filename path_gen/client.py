import time
import serial
from numpy import pi, sin, cos, radians, degrees
from generator import pathGenerator, trajectory, loopTimer
import threading

# Script Config
PLOT_TRAJECTORY = True

SAMPLE_FREQUENCY = 50 # Hz
FILTER_FREQUENCY = 10 # Hz
SERIAL_SPEEDUP = 2 # send serial faster than the sample frequency to ensure buffer filling

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
generator = pathGenerator(SAMPLE_FREQUENCY, FILTER_FREQUENCY)


def trajectory_command(y,p,r,dy,dp,dr):
    # convert to milliradians(/s)
    m_y = int(y*1000)
    m_p = int(p*1000)
    m_r = int(r*1000)
    m_dy = int(dy*1000)
    m_dp = int(dp*1000)
    m_dr = int(dr*1000)
    return "MY{}P{}R{}y{}p{}r{}\n".format(m_y,m_p,m_r,m_dy,m_dp,m_dr)

def position_command(y,p,r):
    # convert to milliradians(/s)
    m_y = int(y*1000)
    m_p = int(p*1000)
    m_r = int(r*1000)
    return "MY{}P{}R{}\n".format(m_y,m_p,m_r)

def send_command(command):
    ser.write(command.encode("ascii"))

def set_state_open_trajectory():
    send_command("O\n")

def set_state_closed_trajectory():
    send_command("O\n")

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
    
def session_timer_callback(traj_y: trajectory, traj_p: trajectory, traj_r: trajectory):
    global session_running, session_loop_timer
    try:
        (y, dy, _) = next(traj_y)
        (p, dp, _) = next(traj_p)
        (r, dr, _) = next(traj_r)
        command = trajectory_command(y,p,r,dy,dp,dr)
        send_command(command)
    except StopIteration:
        session_running = False
        session_loop_timer.stop()
        set_state_idle()

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

def open_loop_test(traj_y, traj_p, traj_r):
    global session_running
    global session_loop_timer
    enable_motors()
    time.sleep(1) # wait for motors to enable
    home_device()
    wait_for_recieved_character('F')
    # Set to Initial position
    set_state_position()
    pos = position_command(traj_y._func[0], traj_p._func[0], traj_r._func[0])
    send_command(pos)
    # print("cmd:", pos)
    wait_for_recieved_character('F')
    time.sleep(1) # wait a second before sending the trajectory
    # Send trajectory
    session_loop_timer = loopTimer(1/(SAMPLE_FREQUENCY*SERIAL_SPEEDUP), session_timer_callback, (traj_y, traj_p, traj_r))
    set_state_open_trajectory()
    session_loop_timer.start()
    session_running = True
    # Plot
    if PLOT_TRAJECTORY:
        if not traj_y.is_zero:
            traj_y.plot()
        if not traj_p.is_zero:
            traj_p.plot()
        if not traj_r.is_zero:
            traj_r.plot()

def main():
    global session_running
    global session_loop_timer
    print("Trajectory Generator for Coaxial Mainipulator")
    printer_thread = threading.Thread(target=read_with_flow_control, daemon=True)
    printer_thread.start()
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
                # commands to run scripts
                    if line == "OY":
                        # period of 4 so that it spends 2 seconds per stride, therefore travels at the speed of the amplitude
                        test_duration = 20
                        traj_y = generator.generate_triangle_trajectory(radians(45),4,test_duration, filter=True)
                        traj_p = generator.generate_zero_trajectory(test_duration)
                        traj_r = generator.generate_zero_trajectory(test_duration)
                        open_loop_test(traj_y, traj_p, traj_r)

                    if line == "OP":
                        test_duration = 20
                        traj_y = generator.generate_zero_trajectory(test_duration)
                        traj_p = generator.generate_triangle_trajectory(radians(30),4,test_duration, filter=True)
                        traj_r = generator.generate_zero_trajectory(test_duration)
                        open_loop_test(traj_y, traj_p, traj_r)

                    if line == "OR":
                        test_duration = 20
                        traj_y = generator.generate_zero_trajectory(test_duration)
                        traj_p = generator.generate_zero_trajectory(test_duration)
                        traj_r = generator.generate_triangle_trajectory(radians(30),4,test_duration, filter=True)
                        open_loop_test(traj_y, traj_p, traj_r)
                    
                    if line == "OYP":
                        # period of 4 so that it spends 2 seconds per stride, therefore travels at the speed of the amplitude
                        test_duration = 20
                        traj_y = generator.generate_triangle_trajectory(radians(45),4,test_duration, filter=True)
                        traj_p = generator.generate_triangle_trajectory(radians(30),4,test_duration, filter=True)
                        traj_r = generator.generate_zero_trajectory(test_duration)
                        open_loop_test(traj_y, traj_p, traj_r)

                    if line == "OYR":
                        test_duration = 20
                        traj_y = generator.generate_triangle_trajectory(radians(45),4,test_duration, filter=True)
                        traj_p = generator.generate_zero_trajectory(test_duration)
                        traj_r = generator.generate_triangle_trajectory(radians(-30),4,test_duration, filter=True)
                        open_loop_test(traj_y, traj_p, traj_r)

                    if line == "OPR":
                        test_duration = 20
                        traj_y = generator.generate_zero_trajectory(test_duration)
                        traj_p = generator.generate_triangle_trajectory(radians(30),4,test_duration, filter=True)
                        traj_r = generator.generate_triangle_trajectory(radians(30),4,test_duration, filter=True)
                        open_loop_test(traj_y, traj_p, traj_r)

                    if line == "OYPR":
                        test_duration = 20
                        traj_y = generator.generate_triangle_trajectory(radians(45),4,test_duration, filter=True)
                        traj_p = generator.generate_triangle_trajectory(radians(30),4,test_duration, filter=True)
                        traj_r = generator.generate_triangle_trajectory(radians(30),4,test_duration, filter=True)
                        open_loop_test(traj_y, traj_p, traj_r)
                    
                    if line == "OPRS":
                        test_duration = 20
                        traj_y = generator.generate_zero_trajectory(test_duration)
                        traj_p = generator.generate_sin_trajectory(radians(20),4,test_duration, filter=True)
                        traj_r = generator.generate_cos_trajectory(radians(20),4,test_duration, filter=True)
                        open_loop_test(traj_y, traj_p, traj_r)
    
        time.sleep(0.01)

main()