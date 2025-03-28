#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

//define structs
typedef struct NODE {
    bool current_state;
    NODE* up;
    NODE* down;
    NODE* left;
    NODE* right;
} NODE;

//assuming we already know the size of the room, just for demonstration
int wall_positions      = 0b11111111111111;
int first_col_positions = 0b00000111000001;
int second_col_positions =0b00000111000001;
int third_col_positions = 0b00000111000001;
int fourth_col_positions =0b00000000000001;
int fifth_col_positions = 0b00000000000001;

//helper functions
int position_states(int row)
{
    int wall_col_value = (wall_positions >> (14 - row)) & 0b1;
    int first_col_value = (first_col_positions >> (14 - row)) & 0b1;
    int second_col_value = (second_col_positions >> (14 - row)) & 0b1;

    int position_states = (wall_positions << 2)| (first_col_positions << 1) | second_col_positions;
    return position_states;
}



//what im envisioning is a function that will read the current output from the detection sensors and can save that output in a grid that can be used for decision making
//general idea


//create new grid node
//send to function to fill out node with up and down data
//
int main(int argc, char *argv[])
{
    //create header node
    NODE* header = malloc(sizeof(NODE));
    header->current_state = NULL;
    header->up = NULL;
    header->down = NULL;
    header->left = NULL;
    header->right = NULL;

    for(int i=0; i < 14; i++) {
        //get testing states
        int states = position_states(i);
        //create current node and below node
        NODE* node = malloc(sizeof(NODE));

        
    }

    return 0;
}