#Connections
from subprocess import Popen, PIPE
from time import sleep

gazebo_sim = "gz sim island.sdf -r".split()
command_list_drone = "sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --map --console --out 0.0.0.0:14551 --out 0.0.0.0:14552 -i0 -L Kattegatt".split()
command_list_boat = "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out 0.0.0.0:14561 --out 0.0.0.0:14562 -i1 -L Kattegatt".split()

Popen(gazebo_sim)
sleep(1)
drone_process = Popen(command_list_drone, stdin=PIPE)
sleep(1)
boat_process = Popen(command_list_boat, stdin=PIPE)

# Try changing so simulations stay up without while true sleep
while True:
    sleep(5)