#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
// #include <semaphore.h>
#include "../controller.h"
#include <stdatomic.h>
#include <stdbool.h>

// ROS2 Stuff
// #include "ros2_control_demo_example_7/r6bot_controller.hpp"

// #include <stddef.h>
// #include <algorithm>
// #include <memory>
// #include <string>
// #include <vector>

// #include "rclcpp/qos.hpp"
// #include "rclcpp/time.hpp"
// #include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
// #include "rclcpp_lifecycle/state.hpp"


#define BUFFER_SIZE 1024

typedef struct {
    atomic_int idx;
    atomic_double values[1]; // u, (dx, da) <- not yet
} Vote;

typedef struct {
    int idx;
    double values[1]; // u, (dx, da) <- not yet
} Vote_NA;

typedef struct {
    atomic_int idx;
    atomic_double values[5]; // x,a,t temp(dx, da)
} State;

typedef struct {
    int idx;
    double values[5]; // x,a,t temp(dx, da)
} State_NA;

typedef struct {
    int idx;
    double values[2]; // dx, da
} Internal;


int main() {
   // open or create the file with the proper permissions
    int fd0 = open("_state", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    // init the size of the file
    lseek (fd0, sizeof(State), SEEK_SET); 
    write (fd0, "", 1); 
    lseek (fd0, 0, SEEK_SET);

    // map the file into memory
    State* state = mmap(NULL, sizeof(State), PROT_WRITE, MAP_SHARED, fd0, 0);
    close(fd0);
    if (state == -1){
        printf("error: %s\n", strerror(errno));
        exit(1);
    }
    printf("%p\n", state);


    // open the internal state file
    int fd1 = open("_internal", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    // init the size of the file
    lseek (fd1, sizeof(Internal), SEEK_SET); 
    write (fd1, "", 1); 
    lseek (fd1, 0, SEEK_SET);

    // map the file into memory
    Internal* internal = mmap(NULL, sizeof(Internal), PROT_WRITE, MAP_SHARED, fd1, 0);
    close(fd1);
    if (internal == -1){
        printf("error: %s\n", strerror(errno));
        exit(1);
    }
    printf("%p\n", internal);

     // open or create the file with the proper permissions
    int fd2 = open("_data0", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
    // init the size of the file
    lseek (fd2, sizeof(Vote), SEEK_SET); 
    write (fd2, "", 1); 
    lseek (fd2, 0, SEEK_SET);

    // map the file into memory
    Vote* data = mmap(NULL, sizeof(Vote), PROT_WRITE, MAP_SHARED, fd2, 0);
    close(fd2);
    if (data == -1){
        printf("error: %s\n", strerror(errno));
        exit(1);
    }
    printf("%p\n", data);

    

    printf("vote: %d\n", sizeof(Vote));
    
    // FIXME maybe init internal here
    atomic_init(&data->idx, 0);
    for (int i = 0; i < 1; i++) { // size of Vote.values
        atomic_init(&data->values[i], 0.0);
    }
    

   
    int myIdx = 0;

    Vote_NA *tmp_vote = malloc(sizeof(Vote_NA));
    State_NA *tmp_state = malloc(sizeof(State_NA));
    Internal *tmp_internal = malloc(sizeof(Internal));

    // tmp_vote->idx = 0;
    // tmp_vote->value = 1.0;

    
    //write_to_buffer(data, *tmp_vote);

    init();

    while(1){

        //sleep(1);
        
        tmp_state->idx = atomic_load_explicit(&state->idx, memory_order_acquire);

        if (tmp_state->idx > myIdx) {
            //printf("new\n");
            for(int i = 0; i < 5; i++) {
                tmp_state->values[i] = atomic_load_explicit(&state->values[i], memory_order_relaxed);
            }
            printf("in: %f,%f,%f,%f,%f\n", tmp_state->values[0], tmp_state->values[1], tmp_state->values[2], tmp_state->values[3], tmp_state->values[4]);
            in[0] = tmp_state->values[0];
            in[1] = tmp_state->values[1];
            in[2] = tmp_state->values[2];
            in[3] = tmp_state->values[3];
            in[4] = tmp_state->values[4];

            step();

            tmp_vote->values[0] = out[0];//tmp_vote->value * 2;
            printf("out: %f\n", tmp_vote->values[0]);
            myIdx = tmp_state->idx;
            tmp_vote->idx = myIdx;
            atomic_store_explicit(&data->values[0], tmp_vote->values[0], memory_order_relaxed);
            atomic_store_explicit(&data->idx, tmp_vote->idx, memory_order_release);
        }
        
        // if(!read_from_buffer(data, tmp_vote)){
        //     printf("ahhhh read\n");
        //     exit(1);
        // }

        // if (tmp_vote->idx < 10)
        //     printf("%f\n", tmp_vote->value);
        // tmp_vote->value = tmp_vote->value * 2;



        // if(!write_to_buffer(data, *tmp_vote)){
        //     printf("ahhhh write\n");
        //     exit(1);
        // }

        //atomic_store_explicit(actuation, *tmp_vote, memory_order_relaxed);


      
    }

  

    return 0;
    
}
