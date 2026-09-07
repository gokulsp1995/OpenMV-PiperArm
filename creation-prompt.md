I want to operate lift buttons with a AgileX Piper L robot arm. The arm has an OpenMV AE3 camera installed 

I need to develop 2 components:

1) OpenMV camera software:
Software that runs on the OpenMV AE3. When triggered this should detect buttons (1) (2) and (3) and send the pose of the button. It should also detect if the button is lit up white (successful press) or dark. It should ignore green light as (1) is always lit green.

It should also detect buttons wiht up arrow (^) and down arrow.

It should use the on-board VL53L8CX 8×8 time‑of‑flight sensor to detect distance of the buttons.

It should use the on-board wifi and connect to the robot's wifi:
SSID: MeiPiehChi
password: robot2robot

The robot's computer is 192.168.123.100 and button detections should be sent to this computer.

OpenMV: https://github.com/openmv/openmv
https://docs.openmv.io/v5.0.0/openmvcam/quickref/openmv-ae3.html

2) AgileX Piper L arm controller that at a start command moves the arm to one of 3 preferred starting positions (which are known: "call-lift", "floor-select-panel1", "floor-select-panel2"). Then uses the data from OpenMV to plan the path of the end effector to the button and push the button. Please add a utility to record these positions by manually moving to the arm to the position then save the positions in yaml files.

Finally moves the arm back to start position and check if press was successfull (button white), if not tries 3 times again.

The AgileX Piper L is conencted to the computer via CAN bus.

AgileX Piper SDK: https://github.com/agilexrobotics/piper_sdk