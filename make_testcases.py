import csv
import sys
import random

LINE_LEN = 7
MIN_IDX = 10

# pass in the controller number
bad = sys.argv[1]

# max_idx = int(sys.argv[2])

with open("missed_" + bad + ".txt") as missed:
    miss_lines = missed.readlines()
    miss = []
    for x in miss_lines:
        idx = x[:-1]
        if int(idx) >= MIN_IDX:
            miss.append(idx)

    miss = [x[:-1] for x in miss_lines]
    print("controller missed: " + str(miss))