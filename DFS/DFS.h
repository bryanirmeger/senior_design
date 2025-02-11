#ifndef DFS_H
#define DFS_H

//enum
enum DCTN {
    UP,
    DOWN,
    LEFT,
    RIGHT
};

//structs
struct _NODE {
    bool obstacle;
    float x_y[2];
    struct _NODE* up;
    struct _NODE* down;
    struct _NODE* left;
    struct _NODE* right;
};
typedef struct _NODE node;

//functions
void set_node(node** center, node** up, node** down, node** left, node** right);
node* insert_node(node** up, node** down, node** left, node** right, bool obstacle);
void destroy_node(node** node);
node* update_node(node** l_node, enum DCTN direction);
node* update_tracked_nodes(node** c_node, enum DCTN direction);
node* map(node** l_up, node** l_down, node** l_left, node** l_right, enum DCTN direction);

#endif