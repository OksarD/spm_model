Repository for Coaxial SPM device Honours Project.

The SPM is designed to replicate angular motion, with a 40 degree slope and infinite yaw rotation.  
The device uses a belt-driven concentric pulley stack to achieve coaxial motion, and features press-fit ball bearings to ensure rigid kinematic motion with no backlash.  

![SPM Coaxial Device set up in Motion Capture Lab](documents/20251024_101631.jpg)

The CAD files (Inventor source files), as well as step files can be found in TowProManipulator_pack.zip  

The device is sent a reference position and velocity (yaw-pitch-roll) in real-time over USB/UART, and controls with a velocity feedforward controller, with 3 PID loops for compensation on each axis. It also has position control for homing and initialisation.  
The device uses an IMU, combined with the stepper motor position in an Extended Kalman Filter to localise the platform.  
The device funcitons can be done through the UART interface. The command protocol is outlined in the documents/Honours_Results_Chapter_durh0014.pdf in the appendix.  

A test script python/client.py can generate test paths and send them to the device, along with automatic homing and intialisation.  
The data used in the results can be found in python/standalone_data and python/vicon_data, and the plots shown in the report can be generated for any of the trials using python/process_vicon.pyand python/process_standlaone.py.  