User Interface:

run path_gen/client.py (CLI)
    A hyphen (-) will bypass commands to the Arduino, if no hyphen is present the commands will be used by the python client.

Python client commands:
    T/t run the test script. This script can generate a sine, cosine or triangular trajectory using the functions in generator.py

Arduino Commands:
    MY#P#R#y#p#r# Move the robot in a specified velocity, dicated by y#p#r# (in integer milliradians/s),
    assuming an operating point dicated by Y#P#R# (in integer milliradians)

    E/D Enable/Disable motors

    S/X Start or Exit the session. A session will use loop timing on the microcontroller level to ensure accurate motor command execution

Example Usage (inside pyhton client):

User Input: -E
User Input: -S
User Input: T

This will enable the motors and loop timing, and run the test trajectory sequence. Note that because it is velocity open loop, there will be some drift in the position.
