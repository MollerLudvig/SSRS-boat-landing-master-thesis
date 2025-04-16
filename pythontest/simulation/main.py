from multiprocessing import Process
import asyncio
# from threading import Thread

import tester
import working
import cookin

# if __name__ == '__main__':
#     Process(target=working.tester).start()
#     #Process(target=positioner_gazebo_box.positioner).start()


async def main():
    task1 = asyncio.create_task(working.tester())
    print("Task 1 started", flush=True)
    # task2 = asyncio.create_task(positioner_gazebo_box.positioner())  # if it's async
    await task1

if __name__ == '__main__':
    print("Starting main process", flush=True)
    asyncio.run(main())