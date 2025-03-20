from multiprocessing import Process
# from threading import Thread

import tester

if __name__ == '__main__':
    Process(target=tester.tester).start()
    #Process(target=positioner_gazebo_box.positioner).start()

