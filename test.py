import numpy as np
import time

period = 0.1
start_time = time.time()
while time.time() < start_time + period:
    time.sleep(period / 10)
end_time = time.time()

print(end_time - start_time)