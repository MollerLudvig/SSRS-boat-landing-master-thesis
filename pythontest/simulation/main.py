from multiprocessing import Process
import matplotlib.pyplot as plt
# from threading import Thread

import tester
import working
import cookin
import real_life_testing

if __name__ == '__main__':
    Process(target=working.tester).start()
    #Process(target=positioner_gazebo_box.positioner).start()

