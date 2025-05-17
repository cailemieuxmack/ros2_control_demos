import csv
import sys
import random
import os

# LINE_LEN = 7
REPAIR_PATH = "."
MIN_IDX = 10
NUM_PASS = 2
NUM_FAIL = 2

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


miss_cases = random.sample(miss, NUM_FAIL)

print(miss_cases)

for i in miss_cases:
    os.system(f"cp results/state_{i} {REPAIR_PATH}/docker/test/n{i}")
    os.system(f"cp results/actuation_{i} {REPAIR_PATH}/docker/test/output.n{i}")

pass_cases = random.sample(passes, NUM_PASS)

print(pass_cases)

for i in pass_cases:
    os.system(f"cp results/state_{i} {REPAIR_PATH}/docker/test/p{i}")
    os.system(f"cp results/actuation_{i} {REPAIR_PATH}/docker/test/output.p{i}")

