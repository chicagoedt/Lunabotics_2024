from phoenix6 import hardware
import typing


'''
doas apt-get update
doas apt-get upgrade
doas apt-get install git
doas apt-get install cmake
git clone git clone https://github.com/CrossTheRoadElec/Phoenix-Linux-SocketCAN-Example.git

# https://v5.docs.ctr-electronics.com/en/stable/ch06b_PrepLinuxRobot.html#why-prepare-linux-robot-controller


'''

#export CTR_TARGET=Hardware
# ran above command in shell beforehand
 
# Export the environment variable so it's persistent in the shell
cancoder = hardware.CANcoder(1, "can0")

while True:

    pos = cancoder.get_position()
    print(f"Positions is {str(pos)} with {pos.timestamp.get_latency()} seconds of latency")

    vel = cancoder.get_velocity()
    # This time wait for the signal to reduce latency

    vel.wait_for_update(0.1)

    print(f"Velocity is {vel} with {vel.timestamp.get_latency()} seconds of latency")
