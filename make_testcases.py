import csv
import sys
import random
import os

# LINE_LEN = 7
REPAIR_PATH = "."
MIN_IDX = 10

# pass in the controller number
bad = sys.argv[1]

max_idx = int(sys.argv[2])

with open("missed_" + bad + ".txt") as missed:
    miss_lines = missed.readlines()
    miss = []
    for x in miss_lines:
        idx = int(x[:-1])
        if idx >= MIN_IDX:
            miss.append(idx)

    # miss = [x[:-1] for x in miss_lines]
    print("controller missed: " + str(miss))


potential_passes = range(MIN_IDX, max_idx)

passes = [x for x in potential_passes if x not in miss]

print(f"passes: {passes}")


miss_cases = random.sample(miss, 2)

for i in miss:
    os.system(f"cp state_{i} {REPAIR_PATH}/docker/test/n{i}")
    os.system(f"cp actuation_{i} {REPAIR_PATH}/docker/test/output.n{i}")