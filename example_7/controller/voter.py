import random
from threading import Timer
import struct
import mmap
import threading
import subprocess
import os
import glob
#import time

num_voters = 1

myIdx = 0

# Define A and trust_scores globally
A = [0,0,0]  
trust_scores = [0.75] * len(A)
trust_scores[0] = 0.4
active_controllers =  [True] * len(A)
#controller_paths = [f'c{i}/controller.c' for i in range(len(A))]

class RepeatTimer(Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)
            
def modify_voter_positions(A):
    # Modify each position in A by adding a random value between -1 and 1
    return [a + random.uniform(-1, 1) for a in A]

def update_trust_scores(votes, accepted_votes, accepted_value):
    # Placeholder for logic to update trust scores
    # Update 'scores' based on 'some_logic'
    global trust_scores, active_controllers
    for subdiv in votes:
        for idx, vote in subdiv:
            if active_controllers[idx]:
                deviation = abs(vote - accepted_value)
                if (idx,vote) in accepted_votes:
                    trust_scores[idx] = min(trust_scores[idx] + 0.5 * ((1 - trust_scores[idx]) / (1 + deviation)), 1)
                else:
                    # take off at most 0.1, scaled by how wrong it is
                    trust_scores[idx] = max(trust_scores[idx] - deviation / 100, 0)
                    write_missed(idx)

def write_missed(idx):
    with open(f'missed_{idx}.txt', 'a') as file:
        file.write(str(myIdx) + "\n")

def delete_missed_files(directory):

    # Construct the path pattern to match all 'missed_*.txt' files in the specified directory

    path_pattern = os.path.join(directory, 'missed_*.txt')

    # Find all files matching the pattern

    files = glob.glob(path_pattern)

    # Remove each file found

    for file in files:

        os.remove(file)

        print(f"Deleted file: {file}")


def check_trust(current_trust_scores, threshold):
    global active_controllers
    for i, score in enumerate(current_trust_scores):
        if active_controllers[i] == True and score < threshold:
            active_controllers[i] = False
            print(f"sending controller {i} for fixing")
            # DEBUG
            exit(1)
            # make a thread to call for repair here on that controller path_to_controller_i
            thread = threading.Thread(target=repair_controller, args=(i,))
            thread.start()
            # assume there is a table/array with paths to controllers
            # thread dies, pass in i

def repair_controller(controller_index):
    #controller_path = controller_paths[controller_index]  # Assuming controller_paths is defined elsewhere
    # Call the external repair script with the path to the controller as an argument
    result = subprocess.run(['bash', 'repair.sh', str(controller_index), str(myIdx)], capture_output=True, text=True)
    if result.returncode == 0:
        # If the script completes successfully, reactivate the controller
        #active_controllers[controller_index] = True
        trust_scores[controller_index] = 0.75 # reset trust score
        print(f"Controller {controller_index} repaired and reactivated.")

        #subprocess.run("cp patches/0.diff patches_"+str(controller_index)+"/0.diff",shell=True, check=True)
        #os._exit(0)

    else:
        # Handle cases where the script fails
        print(f"Failed to repair controller {controller_index}. Error: {result.stderr}")
        os._exit(1)

# FMV
def vote(A, epsilon):
    global active_controllers
    # Initializes a list to hold subdivisions, each subdivision is a list of outputs
    subdivisions = []

    # Iterate over each output in A
    for idx, x in enumerate(A):
        if active_controllers[idx]:
            if type(x) is not float:
                # FIXME this is just for testing and debugging
                # DEBUG should actually handle missed votes better than this
                if(myIdx > 10):
                    trust_scores[idx] -= 0.1
                    write_missed(idx)
                continue
            # A flag to check if x has been added to a subdivision
            added_to_subdivision = False
            
            # Check each existing subdivision to see if x fits into it
            for subdivision in subdivisions:
                # If x is within epsilon of any element in the subdivision, add x to this subdivision
                if any(abs(x - y[1]) <= epsilon for y in subdivision):
                    subdivision.append((idx,x))
                    added_to_subdivision = True
                    break
            
            # If x does not fit into any existing subdivision, create a new one for x
            if not added_to_subdivision:
                subdivisions.append([(idx,x)])

    # print('subd', subdivisions)
    # Find the largest subdivision
    if len(subdivisions) > 0:
        largest_subdivision = max(subdivisions, key=len)

        # Calculate and return the average of the largest subdivision
        average_value = sum(y[1] for y in largest_subdivision) / len(largest_subdivision)
        
        accepted_votes = set(largest_subdivision)
        update_trust_scores(subdivisions, accepted_votes, average_value)
        
        # print('avg val', average_value)
        return average_value
    else:
        print("There are no subdivisions")
        return 0

def driver(data0, actuation):
    global myIdx 
    global A, trust_scores  # Indicate that we're using the global variables
    #A = modify_voter_positions(A)  # Update A with modified positions
    try:
        data0.seek(0)
        (vIdx, v0)  = struct.unpack("id",data0.read(16))
        #A = struct.unpack("d",data0.read(8))[0]
        #print(d.read())
        #print(A)

        if(vIdx >= myIdx):
            A[0] = v0
            print(vIdx,":", A)
            #actuation.write(struct.pack("id", myIdx, A))
            #actuation.write(struct.pack("d", A))
        else:
            A[0] = None
    except struct.error:
        print("could not read file 0")

    try:
        data1.seek(0)
        (vIdx, v1)  = struct.unpack("id",data1.read(16))
        #A = struct.unpack("d",data0.read(8))[0]
        #print(d.read())
        #print(A)

        if(vIdx >= myIdx):
            A[1] = v1
            #print(vIdx,":", A)
            #actuation.write(struct.pack("id", myIdx, A))
            #actuation.write(struct.pack("d", A))
        else:
            A[1] = None
    except struct.error:
        print("could not read file 0")

    try:
        data2.seek(0)
        (vIdx, v2)  = struct.unpack("id",data2.read(16))
        #A = struct.unpack("d",data0.read(8))[0]
        #print(d.read())
        #print(A)

        if(vIdx >= myIdx):
            A[2] = v2
            #print(vIdx,":", A)
            #actuation.write(struct.pack("id", myIdx, A))
            #actuation.write(struct.pack("d", A))
        else:
            A[2] = None
    except struct.error:
        print("could not read file 0")

    epsilon = 0.5 # slightly larger than noise
    
    output = vote(A, epsilon)

    with open("results.csv", 'a') as f:
        f.write(str(output) + "\n")
        # f.flush()
        # os.fsync(f.fileno())
    
    print(A)  # Print updated A for verification
    #print(trust_scores)  # Print updated trust scores for verification
    
    threshold = 0.5
    check_trust(trust_scores, threshold)
    myIdx += 1

    # myIdx = 69
    # output = 69.69

    actuation.seek(0)
    actuation.write(struct.pack("id", myIdx, output))
    


if __name__ == "__main__":
    # Directory containing the files
    directory = "./"  # Update this to your specific folder path if different
    # Delete all 'missed_*.txt' files before running the main function
    delete_missed_files(directory)

    #print(trust_scores)
    with open("_data0", "rb") as d0, open("_data1", "rb") as d1, open("_data2", "rb") as d2, open("_actuation", "w+b") as a:
            #a.write(b"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
            a.write(bytes(16))
            a.seek(0)
            d0.seek(0)
            d1.seek(0)
            d2.seek(0)

            # Create memory maps for data0 and actuation
            data0 = mmap.mmap(d0.fileno(), 0, access=mmap.ACCESS_READ)
            data1 = mmap.mmap(d1.fileno(), 0, access=mmap.ACCESS_READ)
            data2 = mmap.mmap(d2.fileno(), 0, access=mmap.ACCESS_READ)
            actuation = mmap.mmap(a.fileno(), 0, access=mmap.ACCESS_WRITE)

            # print(data0)
            # print(actuation)
            
            # Start timer
            t = RepeatTimer(0.05, driver, [data0, actuation])
            t.start()

    # index of what vote we are on, track which vote indexes it got wrong - to know which ones are test cases, write to a file
    # integrate trust scores
    # detect when a controller breaks/which one
    # add run bash script 
    # integrate into a control loop system
    # input and output 
    # integrate internal state vote???????????
    # of cycles before detection given certain hyperparameters
    # hyperparameters - conservative, moderate, and how long it takes to detect
    # x axis how wrong the controller is y axis how long it took
    # wrong every time, 1/x times and by how much
    