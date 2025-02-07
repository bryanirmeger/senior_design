#include <stdbool.h>
#include <stdlib.h>
#include "DFS.h"

//functions

//creates new node with passed addresses. Then, associates the other nodes with the new one
node* insert_node(node** up, node** down, node** left, node** right, bool obstacle) {
    //create node
    node* new_node = malloc(sizeof(*new_node));
    //set directional nodes for new node
    new_node->up = *up;
    new_node->down = *down;
    new_node->left = *left;
    new_node->right = *right;
    new_node->obstacle = obstacle;
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

//deallocates single node
void destroy_node(node** ref) {
    //desassociate node from all surronding nodes
    if((*ref)->up != NULL){
        (*ref)->up->down = NULL;
    }
    if((*ref)->down != NULL){
        (*ref)->down->up = NULL;
    }
    if((*ref)->left != NULL){
        (*ref)->left->right = NULL;
    }
    if((*ref)->right != NULL){
        (*ref)->right->left = NULL;
    }
    //deallocate node
    free(*ref);
}

node* update_node(node** l_node, enum DCTN direction) {
    //if next node exists, update, if not create and link to last node
    node* next_node = NULL;
    switch(direction) {
        case UP:
            if((*l_node)->up != NULL) {
                next_node = (*l_node)->up;
            }
            else {
                next_node = insert_node(NULL, l_node, NULL, NULL, false);
            }
            break;
        case DOWN:
            if((*l_node)->down != NULL) {
                next_node = (*l_node)->down;
            }
            else {
                next_node = insert_node(l_node, NULL, NULL, NULL, false);
            }
            break;
        case LEFT:
            if((*l_node)->left != NULL) {
                next_node = (*l_node)->left;
            }
            else {
                next_node = insert_node(NULL, NULL, NULL, l_node, false);
            }
            break;
        case RIGHT:
            if((*l_node)->right != NULL) {
                next_node = (*l_node)->right;
            }
            else {
                next_node = insert_node(NULL, NULL, l_node, NULL, false);
            }
            break;
    }
    return next_node;
}

//creates a new node and fits it into map
node* map(node** l_up, node** l_down, node** l_left, node** l_right, enum DCTN direction) {
    return 0;
}