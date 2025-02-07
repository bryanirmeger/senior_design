#include <stdio.h>
#include <stdbool.h>
#include "DFS.h"

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_RESET   "\x1b[0m"

//test function
void test_insert_delete_node_nulls() {
    //test insert with all NULL values
    node* null_node = NULL;
    node* new_node = insert_node(&null_node, &null_node, &null_node, &null_node, false);

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
    node* real_node = insert_node(&null_node, &null_node, &null_node, &null_node, true);
    node* new_node = insert_node(&real_node, &null_node, &null_node, &null_node, false);

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
    node* node_1 = insert_node(&null_node, &null_node, &null_node, &null_node, false);
    node* node_2 = insert_node(&node_1, &null_node, &null_node, &null_node, true);

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
    //create three nodes, node_left is to left of node_middle, node_right is to right of node_middle
    //should create 
    node* null_node = NULL;
    node* node_middle = insert_node(&null_node, &null_node, &null_node, &null_node, false);
    node* node_ = insert_node(&node_1, &null_node, &null_node, &null_node, true);


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

//main
int main(int argc, char *argv[]) {
    //test basic single node functionality
    test_insert_delete_node_nulls();
    test_insert_delete_node_one_real();

    //testing update_node
    test_update_node_one();
    test_update_node_two();

    return 0;
}