from subprocess import Popen
from time import sleep

gazeboSim = Popen("gz sim island.sdf -r".split())
sleep(1)

droneProcess = Popen(
    "sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --map --console "
    "--out 0.0.0.0:14551 --out 0.0.0.0:14552 -i0 -L Kattegatt".split()
)
sleep(1)

boatProcess = Popen(
    "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console "
    "--out 0.0.0.0:14561 --out 0.0.0.0:14562 -i1 -L Kattegatt".split()
)

try:
    while True:
        sleep(5)
except KeyboardInterrupt:
    gazeboSim.terminate()
    droneProcess.terminate()
    boatProcess.terminate()
