#include <stdbool.h>
#include <stdlib.h>
#include "DFS.h"


//function will take data pointers and create the map nodes from the data pointers
//return completed map with the header address stored
map* make_map(float* data_x, float* data_y, bool* data_obs) {
    node* null_node = NULL;

    //thresholds for noise and distance
    float d_threshold = 1;
    float n_threshold = 4;

    //initial values
    enum DCTN direction;
    float x = data_x[0];
    float y = data_y[0];
    map* new_map = NULL;
    node* header = insert_node(&null_node, &null_node, &null_node, &null_node, false, zero_list[]);
    new_map->header = header;
    float* node_data = malloc(sizeof(node_data*2));

    //check that data pointers actually point to allocated data (will be size of buffer)
    if((data_x != NULL) && (data_y != NULL)) {
        //allocate for new_map
        new_map = malloc(sizeof(new_map));
        //local header so that we don't lose original header
        node* local_header = header;

        //check that data is past threshold and reasonable
        //what is reaonable will change as imu testing makes it more clear
        if(((abs(x - data_x[0]) >= d_threshold) || (abs(y - data_y[0]) >= d_threshold)) && ((data_x[0] < n_threshold) && (data_y[0] < n_threshold)) ) {
            //calculate direction
            if(((x - data_x[0]) > 0) && (abs(x - data_x[0]) >= 1)) {
                direction = LEFT;
            }
            else if((x - data_x[0] < 0) && (abs(x - data_x[0]) >= 1)) {
                direction  = RIGHT;
            }
            else if((y - data_y[0] > 0) && (abs(y - data_y[0]) >= 1)) {
                direction = DOWN;
            }
            else if((y - data_y[0] < 0) && (abs(y - data_y[0]) >= 1)) {
                direction = UP;
            }
            
            //update nodes in direction
            node* next_node = update_tracked_nodes(&local_header, direction);
            set_node(&next_node, data_obs, node_data);
            
            //update local header
            local_header = next_node;
        }
    }
    return new_map;
}