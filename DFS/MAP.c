#include <stdbool.h>
#include <stdlib.h>
#include "MAP.h"

//simulation helper functions

//update obstacle and position data as if hardware were changing it
void update_data_ptr(bool* obstacle_list, float (*x_y)[2]) {
    //update list to next value in the list. Assuming list is 10 values deep just for testing
    for(int i = 0; i < 9; i++) {
        obstacle_list[i] = obstacle_list[i+1];
        x_y[i][0] = x_y[i+1][0];
        x_y[i][1] = x_y[i+1][1];
    }
}

//helpers

//set obstacle and x_y values in a node struct
void set_node(node** head, bool obstacle, float* x_y) {
    (*head)->obstacle = obstacle;
    (*head)->x_y = x_y;
}

//free entire row and each node above a header
void destroy_row_and_above(node** header) {
    //get intial starting values
    node* start = *header;
    node* next = start->right;
    node* up = start->up;

    while(start->left != NULL) {
        start = start->left;
    }

    while(start != NULL) {
        //destroy start
        destroy_node(&start);
        //check if up is null
        if(up != NULL) {
            destroy_row_and_above(&up);
        }
        //update values
        start = next;
        if(start != NULL) {
            next = start->right;
            up = start->up;
        }
    }
}

//set head of process to the next node in some direction. if this node does not exist, create a new node
node* update_node(node** l_node, enum DCTN direction) {
    //if next node exists, update, if not create and link to last node
    float empty_list[] = {0,0};
    node* null_node = NULL;
    node* next_node = NULL;
    switch(direction) {
        case UP:
            if((*l_node)->up != NULL) {
                next_node = (*l_node)->up;
            }
            else {
                next_node = insert_node(&null_node, l_node, &null_node, &null_node, false, empty_list);
            }
            break;
        case DOWN:
            if((*l_node)->down != NULL) {
                next_node = (*l_node)->down;
            }
            else {
                next_node = insert_node(l_node, &null_node, &null_node, &null_node, false, empty_list);
            }
            break;
        case LEFT:
            if((*l_node)->left != NULL) {
                next_node = (*l_node)->left;
            }
            else {
                next_node = insert_node(&null_node, &null_node, &null_node, l_node, false, empty_list);
            }
            break;
        case RIGHT:
            if((*l_node)->right != NULL) {
                next_node = (*l_node)->right;
            }
            else {
                next_node = insert_node(&null_node, &null_node, l_node, &null_node, false, empty_list);
            }
            break;
    }
    return next_node;
}

//deallocates single node
void destroy_node(node** ref_node) {
    node* null_node = NULL;
    //desassociate node from all surronding nodes
    if(*ref_node != NULL) {
        //disociate_nodes from surrounding nodes
        //dissociate_node(ref_node);
            //get addresses of surrounding nodes
        node* up = (*ref_node)->up;
        node* down = (*ref_node)->down;
        node* left = (*ref_node)->left;
        node* right = (*ref_node)->right;
    
            //set relevant node references to null
        if(up != NULL){
            (up->down) = null_node;
        }
        if(down != NULL){
            (down->up) = null_node;
        }
        if(left != NULL){
            (left->right) = null_node;
        }
        if(right != NULL){
            right->left = null_node;
        }

        //deallocate node
        free(*ref_node);
        *ref_node = null_node;
    }
}

//functions 

//creates new node with passed addresses. Then, associates the other nodes with the new one
node* insert_node(node** up, node** down, node** left, node** right, bool obstacle, float* x_y) {
    //create node
    node* new_node = malloc(sizeof(*new_node));
    //set directional nodes for new node
    new_node->up = *up;
    new_node->down = *down;
    new_node->left = *left;
    new_node->right = *right;
    set_node(&new_node, obstacle, x_y);
    //set data for other nodes. check if null, first
    if(*up != NULL) {
        (*up)->down = new_node;
    }
    if(*down != NULL) {
        (*down)->up = new_node;
    }
    if(*left != NULL) {
        (*left)->right = new_node;
    }
    if(*right != NULL) {
        (*right)->left = new_node;
    }
    //return node
    return new_node;
}

node* update_tracked_nodes(node** c_node, enum DCTN direction) {
    float empty_list[] = {0,0};

    node* up = (*c_node)->up;
    node* down = (*c_node)->down;
    node* left = (*c_node)->left;
    node* right = (*c_node)->right;

    //update nodes around the current position. if null, dont bother updating
    if(up != NULL) {
        up = update_node(&up, direction);
    }
    if(down != NULL) {
        down = update_node(&down, direction);
    }
    if(left != NULL) {
        left = update_node(&left, direction);
    }
    if(right != NULL) {
        right = update_node(&right, direction);
    }

    node* new_c_node = NULL;
    //set surrounding nodes to current node and sets last value in opposite direction to be last value
    switch(direction) {
        case UP:
            new_c_node = insert_node(&up, c_node, &left, &right, false, empty_list);
        break;
        case DOWN:
            new_c_node = insert_node(c_node, &down, &left, &right, false, empty_list);
        break;
        case LEFT:
           new_c_node = insert_node(&up, &down, &left, c_node, false, empty_list);
        break;
        case RIGHT:
            new_c_node = insert_node(&up, &down, c_node, &right, false, empty_list);
        break;
    }

    return new_c_node;
}

//destroy all nodes attached to head
void destroy_map(node** head) {
    node* start = *head;
    node* next = start->right;
    node* up = start->up;

    //move to left most node of row
    while(start->left != NULL) {
        start = start->left;
    }
    //save row below leftmost row node
    node* next_row = start->down;

    //go down row and delete 
    while(start != NULL) {
        //destroy start
        destroy_node(&start);
        //check if up is null
        if(up != NULL) {
            destroy_row_and_above(&up);
        }
        //update values
        start = next;
        if(start != NULL) {
            next = start->right;
            up = start->up;
        }
    }
    if(next_row != NULL) {
        destroy_map(&next_row);
    }

}


/*map* make_map(float* data_x, float* data_y, bool* data_obs) {
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
}*/