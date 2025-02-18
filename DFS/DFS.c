#include <stdbool.h>
#include <stdlib.h>
#include "DFS.h"

//helper
void set_node(node** center, node** up, node** down, node** left, node** right) {
    (*center)->up = *up;
    (*center)->down = *down;
    (*center)->left = *left;
    (*center)->right = *right;
}

void dissociate_node(node** ref_node) {
    node* null_node = NULL;
    if(*ref_node != NULL) {
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
    }
}

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

        //deallocate node
        free(*ref_node);
        *ref_node = null_node;
    }
}

//creates new node with passed addresses. Then, associates the other nodes with the new one
node* insert_node(node** up, node** down, node** left, node** right, bool obstacle, float x_y[]) {
    //create node
    node* new_node = malloc(sizeof(*new_node));
    //set directional nodes for new node
    new_node->up = *up;
    new_node->down = *down;
    new_node->left = *left;
    new_node->right = *right;
    new_node->obstacle = obstacle;
    new_node->traversed = false;
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
    if(*head != NULL) {
        (*head)->traversed = true;
        if(((*head)->up != NULL) && (((*head)->up)->traversed == false)) {
            destroy_map(&((*head)->up));
        }
        if(((*head)->down != NULL) && (((*head)->down)->traversed == false)) {
            destroy_map(&((*head)->down));
        }
        if(((*head)->left != NULL) && (((*head)->left)->traversed == false)) {
            destroy_map(&((*head)->left));
        }
        if(((*head)->right != NULL) && (((*head)->right)->traversed == false)) {
            destroy_map(&((*head)->right));
        }
        destroy_node(head);
    }
}

//creates a new node and fits it into map
node* map(node** l_up, node** l_down, node** l_left, node** l_right, enum DCTN direction) {
    return 0;
}