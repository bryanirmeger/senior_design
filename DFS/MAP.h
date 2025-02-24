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
    bool traversed;
    float* x_y;
    struct _NODE* up;
    struct _NODE* down;
    struct _NODE* left;
    struct _NODE* right;
};
typedef struct _NODE node;

typedef struct _MAP {
    node* header;
    float* data_x;
    float* data_y;

}map;

//functions
void update_data_ptr(bool (*obstacle_list), float (*x_y)[2]);
void set_node(node** head, bool obstacle, float* x_y);
void destroy_row_and_above(node** node);
node* insert_node(node** up, node** down, node** left, node** right, bool obstacle, float* x_y);
void destroy_node(node** node);
node* update_node(node** l_node, enum DCTN direction);
node* update_tracked_nodes(node** c_node, enum DCTN direction);
void destroy_map(node** head);
map* make_map(float* data_x, float* data_y, bool* data_obs);

#endif