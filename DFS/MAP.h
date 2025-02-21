#ifndef MAP_H
#define MAP_H

#include "DFS.h"

//data structures
typedef struct _MAP {
    node* header;
    float* data_x;
    float* data_y;

}map;

//functions
map* make_map(float* data_x, float* data_y, bool* data_obs);

#endif