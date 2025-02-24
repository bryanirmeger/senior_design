#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include "MAP.h"

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_RESET   "\x1b[0m"

//test function
void test_insert_delete_node_nulls() {
    //test insert with all NULL values
    
    node* null_node = NULL;
    node* new_node = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);

    //test output
    printf(ANSI_COLOR_RESET "\n");
    if(new_node->up == NULL) {
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else if(new_node->up != NULL) {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }

    if(new_node->down == NULL) {
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else if(new_node->down != NULL) {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    if(new_node->left == NULL) {
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else if(new_node->left != NULL) {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    if(new_node->right == NULL) {
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else if(new_node->right != NULL) {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    if(new_node->obstacle == false) {
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else if(new_node->obstacle != true) {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }

    printf(ANSI_COLOR_RESET "\n");

    //deallocate node
    destroy_node(&new_node);
}

void test_insert_delete_node_one_real() {
    node* null_node = NULL;
    
    //create a node that will be placed at the top of the new node. This should make real->down = new_node and vice versa
    node* real_node = insert_node(&null_node, &null_node, &null_node, &null_node, NULL, NULL);
    node* new_node = insert_node(&real_node, &null_node, &null_node, &null_node, NULL, NULL);

    //tests
    printf(ANSI_COLOR_RESET "\n");

    if(new_node->up == real_node) {
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else if(new_node->up != real_node) {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }

    if(real_node->down == new_node) {
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else if(real_node->down != new_node) {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }

    printf(ANSI_COLOR_RESET "\n");
    
    //deallocate two nodes

    destroy_node(&real_node);
    destroy_node(&new_node);
}

void test_update_node_one() {
    //create two nodes, node_1 is above node_2, or move DOWN from 1 to two
    
    node* null_node = NULL;
    node* node_1 = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);
    node* node_2 = insert_node(&node_1, &null_node, &null_node, &null_node,  NULL, NULL);

    //create header at node_1
    node* move_node = node_1;
    move_node = update_node(&move_node, DOWN);

    //tests
    printf(ANSI_COLOR_RESET "\n");

    //sanity check, make sure they linked
    if((node_1->down == node_2) || (node_2->up == node_1)) {
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else if((node_1->down != node_2) || (node_2->up != node_1)) {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }

    if(move_node == node_2) {
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }

    printf(ANSI_COLOR_RESET "\n");

    //deallocate memory
    destroy_node(&node_1);
    destroy_node(&node_2);
}

void test_update_node_two() {
    //create three nodes, node_c is center, node-r is to the right of node_c
    
    node* null_node = NULL;
    node* node_c = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);
    node* node_r = insert_node(&null_node, &null_node, &node_c, &null_node,  NULL, NULL);

    //nodes should be attached to each other with insert_node

    //create header at node_1
    node* move_node = node_c;
    move_node = update_tracked_nodes(&move_node, DOWN);

    //tests
    printf(ANSI_COLOR_RESET "\n");

    //sanity check, make sure they linked
    if((move_node->right == node_r->down) && (node_r->down->left == move_node)) { 
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    if((node_c->right == node_r) && (node_r->left == node_c) && (node_c->down == move_node) && (move_node->up == node_c)) {
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    
    printf(ANSI_COLOR_RESET "\n");

    //deallocate memory
    destroy_map(&node_c);
}

void test_destroy_map_one() {
    
    node* null_node = NULL;

    //create single node
    node* node_c = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);
    //clear single node
    destroy_map(&node_c);

    printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    printf(ANSI_COLOR_RESET "\n");
}

void test_destroy_map_two() {
    
    node* null_node = NULL;

    //create single node
    node* node_c = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);
    node* node_r = insert_node(&null_node, &null_node, &null_node, &node_c,  NULL, NULL);
    //clear single node
    destroy_map(&node_r);

    printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    printf(ANSI_COLOR_RESET "\n");
}

void test_destroy_map_cluster() {
    
    node* null_node = NULL;

    //create single node
    node* node_r = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);
    node* node_u = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);
    node* node_c = insert_node(&node_u, &null_node, &null_node, &node_r,  NULL, NULL);
    //clear single node
    destroy_map(&node_c);

    printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    printf(ANSI_COLOR_RESET "\n");
}

void test_destroy_map_loop() {
    
    node* null_node = NULL;

    //create single node
    node* node_ur = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);
    node* node_u = insert_node(&null_node, &null_node, &null_node, &node_ur,  NULL, NULL);
    node* node_r = insert_node(&node_ur, &null_node, &null_node, &null_node,  NULL, NULL);
    node* node_c = insert_node(&node_u, &null_node, &null_node, &node_r,  NULL, NULL);
    //clear single node
    destroy_map(&node_c);

    printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    printf(ANSI_COLOR_RESET "\n");
}

void test_destroy_map_alot() {
    
    node* null_node = NULL;

    node* uuuun_1 = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);
    
    node* uuun_2 = insert_node(&uuuun_1, &null_node, &null_node, &null_node,  NULL, NULL);
    node* uuun_1 = insert_node(&null_node, &null_node, &null_node, &uuun_2,  NULL, NULL);

    node* uun_1 = insert_node(&uuun_1, &null_node, &null_node, &null_node,  NULL, NULL);

    node* un_5 = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);
    node* un_4 = insert_node(&null_node, &null_node, &null_node, &un_5,  NULL, NULL);
    node* un_3 = insert_node(&null_node, &null_node, &null_node, &un_4,  NULL, NULL);
    node* un_2 = insert_node(&uun_1, &null_node, &null_node, &un_3,  NULL, NULL);

    node* un_1 = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);

    node* n_4 = insert_node(&un_3, &null_node, &null_node, &null_node,  NULL, NULL); 
    node* n_3 = insert_node(&un_2, &null_node, &null_node, &n_4,  NULL, NULL);  
    node* n_2 = insert_node(&null_node, &null_node, &null_node, &n_3,  NULL, NULL);    
    node* head = insert_node(&un_1, &null_node, &null_node, &n_2,  NULL, NULL);

    destroy_map(&head);

    printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    printf(ANSI_COLOR_RESET "\n");
}

void test_destroy_node_and_above_one() {
    
    node* null_node = NULL;
    //create node
    node* c = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);   

    destroy_row_and_above(&c);

    printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    printf(ANSI_COLOR_RESET "\n");
}

void test_destroy_node_and_above_two() {
    
    node* null_node = NULL;
    //create node
    node* n_2 = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);   
    node* head = insert_node(&null_node, &null_node, &null_node, &n_2,  NULL, NULL);

    destroy_row_and_above(&head);

    printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    printf(ANSI_COLOR_RESET "\n");
}

void test_destroy_node_and_above_one_row() {
    
    node* null_node = NULL;
    //create row of nodes
    node* n_7 = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);  
    node* n_6 = insert_node(&null_node, &null_node, &null_node, &n_7,  NULL, NULL);  
    node* n_5 = insert_node(&null_node, &null_node, &null_node, &n_6,  NULL, NULL);  
    node* n_4 = insert_node(&null_node, &null_node, &null_node, &n_5,  NULL, NULL);  
    node* n_3 = insert_node(&null_node, &null_node, &null_node, &n_4,  NULL, NULL);  
    node* n_2 = insert_node(&null_node, &null_node, &null_node, &n_3,  NULL, NULL);   
    node* head = insert_node(&null_node, &null_node, &null_node, &n_2,  NULL, NULL);

    destroy_row_and_above(&head);

    printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    printf(ANSI_COLOR_RESET "\n");
}

void test_destroy_node_and_above_one_row_one_above() {
    
    node* null_node = NULL;
    //create row of nodes
    node* uun_1 = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL); 
    node* un_2 = insert_node(&uun_1, &null_node, &null_node, &null_node,  NULL, NULL); 
    node* un_1 = insert_node(&null_node, &null_node, &null_node, &un_2,  NULL, NULL); 
    node* n_7 = insert_node(&un_2, &null_node, &null_node, &null_node,  NULL, NULL);  
    node* n_6 = insert_node(&un_1, &null_node, &null_node, &n_7,  NULL, NULL);    
    node* head = insert_node(&null_node, &null_node, &null_node, &n_6,  NULL, NULL);

    destroy_row_and_above(&head);

    printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    printf(ANSI_COLOR_RESET "\n");
}

void test_destroy_node_and_above_alot() {
    
    node* null_node = NULL;

    node* uuuun_1 = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);
    
    node* uuun_2 = insert_node(&uuuun_1, &null_node, &null_node, &null_node,  NULL, NULL);
    node* uuun_1 = insert_node(&null_node, &null_node, &null_node, &uuun_2,  NULL, NULL);

    node* uun_1 = insert_node(&uuun_1, &null_node, &null_node, &null_node,  NULL, NULL);

    node* un_5 = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);
    node* un_4 = insert_node(&null_node, &null_node, &null_node, &un_5,  NULL, NULL);
    node* un_3 = insert_node(&null_node, &null_node, &null_node, &un_4,  NULL, NULL);
    node* un_2 = insert_node(&uun_1, &null_node, &null_node, &un_3,  NULL, NULL);

    node* un_1 = insert_node(&null_node, &null_node, &null_node, &null_node,  NULL, NULL);

    node* n_4 = insert_node(&un_3, &null_node, &null_node, &null_node,  NULL, NULL); 
    node* n_3 = insert_node(&un_2, &null_node, &null_node, &n_4,  NULL, NULL);  
    node* n_2 = insert_node(&null_node, &null_node, &null_node, &n_3,  NULL, NULL);    
    node* head = insert_node(&un_1, &null_node, &null_node, &n_2,  NULL, NULL);

    destroy_row_and_above(&head);

    printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    printf(ANSI_COLOR_RESET "\n");
}

void test_set_node_direct() {
    node* null_node = NULL;
    float x_y[2] = {0,1};
    bool obs = true;
    node* n = insert_node(&null_node, &null_node, &null_node, &null_node, NULL, NULL);
    set_node(&n, obs, x_y);

    //tests
    printf(ANSI_COLOR_RESET "\n");

    //sanity check, make sure they linked
    if(n->x_y == x_y) { 
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    if(n->obstacle == obs) {
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    if((n->x_y)[0] == x_y[0]) { 
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    if((n->x_y)[1] == x_y[1]) { 
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    
    printf(ANSI_COLOR_RESET "\n");

    //deallocate
    destroy_map(&n);
}

void test_set_node_through_insert_node() {
    node* null_node = NULL;
    float x_y[2] = {0,1};
    bool obs = true;
    node* n = insert_node(&null_node, &null_node, &null_node, &null_node, obs, x_y);

    if(n->x_y == x_y) { 
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    if(n->obstacle == obs) {
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    if((n->x_y)[0] == x_y[0]) { 
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    if((n->x_y)[1] == x_y[1]) { 
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
    
    printf(ANSI_COLOR_RESET "\n");

    //deallocate
    destroy_map(&n);
}

void test_update_data_ptr_pass_data() {
    //create test data
    float data_x_y[10][2] = {{0,0},{0,1},{0,2}, {0,3},{0,4},{0,5}, {0,6},{0,7},{0,8}, {0,9}};
    bool data_obs[10] = {false, false, false, false, false, false, false, false, false, false};

    if((data_x_y[0][0] == 0) && (data_x_y[0][1] == 0)) { 
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }

    update_data_ptr(data_obs, data_x_y);
    //1
    update_data_ptr(data_obs, data_x_y);
    //2
    update_data_ptr(data_obs, data_x_y);
    //3
    update_data_ptr(data_obs, data_x_y);
    //4
    update_data_ptr(data_obs, data_x_y);
    //5
    update_data_ptr(data_obs, data_x_y);
    //6
    update_data_ptr(data_obs, data_x_y);
    //7
    update_data_ptr(data_obs, data_x_y);
    //8
    update_data_ptr(data_obs, data_x_y);
    //9
    if((data_x_y[0][0] == 0) && (data_x_y[0][1] == 9)) { 
        printf(ANSI_COLOR_GREEN "Test Passed: %s\n", __func__);
    }
    else {
        printf(ANSI_COLOR_RED "Test Failed: %s\n", __func__);
    }
}

//main
int main(int argc, char *argv[]) {
    //test basic single node functionality
    test_insert_delete_node_nulls();
    test_insert_delete_node_one_real();

    //testing update_node
    test_update_node_one();
    test_update_node_two();

    //testing destroy_map
    test_destroy_map_one();
    test_destroy_map_two();
    test_destroy_map_cluster();
    test_destroy_map_loop();
    test_destroy_map_alot();

    //testing destory_row_and_above
    test_destroy_node_and_above_one();
    test_destroy_node_and_above_two();
    test_destroy_node_and_above_one_row();
    test_destroy_node_and_above_one_row_one_above();
    test_destroy_node_and_above_alot();

    //test new set_node functionality
    test_set_node_direct();
    test_set_node_through_insert_node();

    //test simulation funcs
    test_update_data_ptr_pass_data();

    return 0;
}