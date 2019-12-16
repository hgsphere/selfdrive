import multiprocessing as mp
import queue
import numpy as np
from time import time
import sys
import fmq

q1 = mp.Queue(10)
q2 = fmq.Queue(10)
q3 = queue.Queue(10)

# uncomment this line to switch the order
# q1, q2 = q2, q1

a = np.zeros((50, 256, 256, 3))
a_size = sys.getsizeof(a)
print('Object size: %d bytes = %dKB = %dMB' % (a_size, a_size / 1024, a_size / 1024 / 1024))

for i in range(10):
    q1.put(np.array(a))
    q2.put(np.array(a))
    q3.put(np.array(a))

# mp queue get
for i in range(5):
    st = time()
    b = q1.get()
    print('mp get() a time', time() - st)

# fmq queue get
for i in range(5):
    st = time()
    b = q2.get()
    print('fmq get() a time', time() - st)

for i in range(5):
    st = time()
    b = q3.get()
    print('queue get() a time', time() - st)


if __name__ == '__main__':
    print("\n !!! This is a test file that was never used in the final code base. !!!\n")
