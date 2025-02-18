#include <stdio.h>
#include <stdbool.h>
#include "DFS.h"

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_RESET   "\x1b[0m"

//test function
void test_insert_delete_node_nulls() {
    //test insert with all NULL values
    float empty_list[] = {0,0};
    node* null_node = NULL;
    node* new_node = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);

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
    float empty_list[] = {0,0};
    //create a node that will be placed at the top of the new node. This should make real->down = new_node and vice versa
    node* real_node = insert_node(&null_node, &null_node, &null_node, &null_node, true, empty_list);
    node* new_node = insert_node(&real_node, &null_node, &null_node, &null_node, false, empty_list);

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
    float empty_list[] = {0,0};
    node* null_node = NULL;
    node* node_1 = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);
    node* node_2 = insert_node(&node_1, &null_node, &null_node, &null_node, true, empty_list);

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
    float empty_list[] = {0,0};
    node* null_node = NULL;
    node* node_c = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);
    node* node_r = insert_node(&null_node, &null_node, &node_c, &null_node, true, empty_list);

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
    float empty_list[] = {0,0};
    node* null_node = NULL;

    //create single node
    node* node_c = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);
    //clear single node
    destroy_map(&node_c);
}

void test_destroy_map_two() {
    float empty_list[] = {0,0};
    node* null_node = NULL;

    //create single node
    node* node_c = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);
    node* node_r = insert_node(&null_node, &null_node, &null_node, &node_c, false, empty_list);
    //clear single node
    destroy_map(&node_r);
}

void test_destroy_map_cluster() {
    float empty_list[] = {0,0};
    node* null_node = NULL;

    //create single node
    node* node_r = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);
    node* node_u = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);
    node* node_c = insert_node(&node_u, &null_node, &null_node, &node_r, false, empty_list);
    //clear single node
    destroy_map(&node_c);
}

void test_destroy_map_loop() {
    float empty_list[] = {0,0};
    node* null_node = NULL;

    //create single node
    node* node_ur = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);
    node* node_u = insert_node(&null_node, &null_node, &null_node, &node_ur, false, empty_list);
    node* node_r = insert_node(&node_ur, &null_node, &null_node, &null_node, false, empty_list);
    node* node_c = insert_node(&node_u, &null_node, &null_node, &node_r, false, empty_list);
    //clear single node
    destroy_map(&node_c);
}

void test_destroy_node_and_above_one() {
    float empty_list[] = {0,0};
    node* null_node = NULL;
    //create node
    node* c = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);   

    destroy_row_and_above(&c);
}

void test_destroy_node_and_above_two() {
    float empty_list[] = {0,0};
    node* null_node = NULL;
    //create node
    node* n_2 = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);   
    node* head = insert_node(&null_node, &null_node, &null_node, &n_2, false, empty_list);

    destroy_row_and_above(&head);
}

void test_destroy_node_and_above_one_row() {
    float empty_list[] = {0,0};
    node* null_node = NULL;
    //create row of nodes
    node* n_7 = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);  
    node* n_6 = insert_node(&null_node, &null_node, &null_node, &n_7, false, empty_list);  
    node* n_5 = insert_node(&null_node, &null_node, &null_node, &n_6, false, empty_list);  
    node* n_4 = insert_node(&null_node, &null_node, &null_node, &n_5, false, empty_list);  
    node* n_3 = insert_node(&null_node, &null_node, &null_node, &n_4, false, empty_list);  
    node* n_2 = insert_node(&null_node, &null_node, &null_node, &n_3, false, empty_list);   
    node* head = insert_node(&null_node, &null_node, &null_node, &n_2, false, empty_list);

    destroy_row_and_above(&head);
}

void test_destroy_node_and_above_one_row_one_above() {
    float empty_list[] = {0,0};
    node* null_node = NULL;
    //create row of nodes
    node* uun_1 = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list); 
    node* un_2 = insert_node(&uun_1, &null_node, &null_node, &null_node, false, empty_list); 
    node* un_1 = insert_node(&null_node, &null_node, &null_node, &un_2, false, empty_list); 
    node* n_7 = insert_node(&un_2, &null_node, &null_node, &null_node, false, empty_list);  
    node* n_6 = insert_node(&un_1, &null_node, &null_node, &n_7, false, empty_list);    
    node* head = insert_node(&null_node, &null_node, &null_node, &n_6, false, empty_list);

    destroy_row_and_above(&head);
}

void test_destroy_node_and_above_alot() {
    float empty_list[] = {0,0};
    node* null_node = NULL;

    node* uuuun_1 = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);
    
    node* uuun_2 = insert_node(&uuuun_1, &null_node, &null_node, &null_node, false, empty_list);
    node* uuun_1 = insert_node(&null_node, &null_node, &null_node, &uuun_2, false, empty_list);

    node* uun_1 = insert_node(&uuun_1, &null_node, &null_node, &null_node, false, empty_list);

    node* un_5 = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);
    node* un_4 = insert_node(&null_node, &null_node, &null_node, &un_5, false, empty_list);
    node* un_3 = insert_node(&null_node, &null_node, &null_node, &un_4, false, empty_list);
    node* un_2 = insert_node(&uun_1, &null_node, &null_node, &un_3, false, empty_list);

    node* un_1 = insert_node(&null_node, &null_node, &null_node, &null_node, false, empty_list);

    node* n_4 = insert_node(&un_3, &null_node, &null_node, &null_node, false, empty_list); 
    node* n_3 = insert_node(&un_2, &null_node, &null_node, &n_4, false, empty_list);  
    node* n_2 = insert_node(&null_node, &null_node, &null_node, &n_3, false, empty_list);    
    node* head = insert_node(&un_1, &null_node, &null_node, &n_2, false, empty_list);

    destroy_row_and_above(&head);
}

//main
int main(int argc, char *argv[]) {
    //test basic single node functionality
    //test_insert_delete_node_nulls();
    //test_insert_delete_node_one_real();

    //testing update_node
    //test_update_node_one();
    //test_update_node_two();

    //testing destroy_map
    //test_destroy_map_one();
    //test_destroy_map_two();
    //test_destroy_map_cluster();
    //test_destroy_map_loop();

    //test_destroy_node_and_above_one();
    //test_destroy_node_and_above_two();
    //test_destroy_node_and_above_one_row();
    //test_destroy_node_and_above_one_row_one_above();
    test_destroy_node_and_above_alot();

    return 0;
}