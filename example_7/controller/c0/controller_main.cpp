#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <iostream>
#include <errno.h>
#include <string.h>
#include <atomic>
#include <cstdint>

extern "C" {
   #include "../controller.h"
}

#define BUFFER_SIZE 1024

struct Vote {
    int idx;
    double values[1]; // u, (dx, da) <- not yet
};

// struct Vote_NA {
//     int idx;
//     double values[1]; // u, (dx, da) <- not yet
// };

struct State {
    int idx;
    std::atomic<double> values[5]; // x,a,t temp(dx, da)
};

// struct State_NA {
//     int idx;
//     double values[5]; // x,a,t temp(dx, da)
// };

struct Internal {
    int idx;
    double values[2]; // dx, da
};

int main() {
    // open or create the file with the proper permissions
    int fd0 = open("_state", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    // init the size of the file
    lseek(fd0, sizeof(State), SEEK_SET);
    write(fd0, "", 1);
    lseek(fd0, 0, SEEK_SET);

    // map the file into memory
    State* state = static_cast<State*>(mmap(NULL, sizeof(State), PROT_WRITE, MAP_SHARED, fd0, 0));
    close(fd0);
    if (state == MAP_FAILED) {
        std::cerr << "error: " << strerror(errno) << std::endl;
        exit(1);
    }
    std::cout << state << std::endl;

    // open the internal state file
    int fd1 = open("_internal", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    // init the size of the file
    lseek(fd1, sizeof(Internal), SEEK_SET);
    write(fd1, "", 1);
    lseek(fd1, 0, SEEK_SET);

    // map the file into memory
    Internal* internal = static_cast<Internal*>(mmap(NULL, sizeof(Internal), PROT_WRITE, MAP_SHARED, fd1, 0));
    close(fd1);
    if (internal == reinterpret_cast<Internal*>(-1)) {
        std::cerr << "error: " << strerror(errno) << std::endl;
        exit(1);
    }
    std::cout << internal << std::endl;

    // open or create the file with the proper permissions
    int fd2 = open("_data0", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    // init the size of the file
    lseek(fd2, sizeof(Vote), SEEK_SET);
    write(fd2, "", 1);
    lseek(fd2, 0, SEEK_SET);

    // map the file into memory
    Vote* data = static_cast<Vote*>(mmap(NULL, sizeof(Vote), PROT_WRITE, MAP_SHARED, fd2, 0));
    close(fd2);
    if (data == MAP_FAILED) {
        std::cerr << "error: " << strerror(errno) << std::endl;
        exit(1);
    }
    std::cout << data << std::endl;

    std::cout << "vote: " << sizeof(Vote) << std::endl;

    // FIXME maybe init internal here
    data->idx = 0; //.store(0, std::memory_order_relaxed);
    for (int i = 0; i < 1; i++) { // size of Vote.values
        data->values[i] = 0.0; //.store(0.0, std::memory_order_relaxed);
    }

    int myIdx = -1;

    Vote* tmp_vote = static_cast<Vote_NA*>(malloc(sizeof(Vote_NA)));
    State* tmp_state = static_cast<State_NA*>(malloc(sizeof(State_NA)));
    Internal* tmp_internal = static_cast<Internal*>(malloc(sizeof(Internal)));

    //std::cout << "calling init" << std::endl;
    init();
    //std::cout << "returned from init" << std::endl;

    while (true) {
        tmp_state->idx = state->idx; //.load(std::memory_order_acquire);

        std::cout << "Idx recieved: " << tmp_state->idx << std::endl;


        if (tmp_state->idx > myIdx) {
            for (int i = 0; i < 5; i++) {
                tmp_state->values[i] = state->values[i]; //.load(std::memory_order_relaxed);
            }
            std::cout << "in: " << tmp_state->values[0] << "," << tmp_state->values[1] << "," << tmp_state->values[2] << "," << tmp_state->values[3] << "," << tmp_state->values[4] << std::endl;
            in[0] = tmp_state->values[0];
            in[1] = tmp_state->values[1];
            in[2] = tmp_state->values[2];
            in[3] = tmp_state->values[3];
            in[4] = tmp_state->values[4];

            step();

            tmp_vote->values[0] = out[0];
            std::cout << "out: " << tmp_vote->values[0] << std::endl;
            myIdx = tmp_state->idx;
            tmp_vote->idx = myIdx;
            data->values[0] = tmp_vote->values[0]; //.store(tmp_vote->values[0], std::memory_order_relaxed);
            data->idx = tmp_vote->idx; //.store(tmp_vote->idx, std::memory_order_release);
        }
    }

    return 0;
}