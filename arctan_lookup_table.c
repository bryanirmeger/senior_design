#include <stdio.h>
#include <math.h>

#define X_RES 100  // Resolution for x
#define Y_RES 100  // Resolution for y
#define X_MIN -10  // Minimum x value
#define X_MAX  10  // Maximum x value
#define Y_MIN -10  // Minimum y value
#define Y_MAX  10  // Maximum y value

double lookup_table[Y_RES][X_RES];  // Lookup table for theta

void create_lookup_table() {
    double x_step = (X_MAX - X_MIN) / (X_RES - 1);  // Step size for x
    double y_step = (Y_MAX - Y_MIN) / (Y_RES - 1);  // Step size for y

    for (int i = 0; i < Y_RES; i++) {
        double y = Y_MIN + i * y_step;
        for (int j = 0; j < X_RES; j++) {
            double x = X_MIN + j * x_step;

            if (x == 0 && y == 0) {
                lookup_table[i][j] = 0;  // Define theta(0, 0) as 0 (or handle as needed)
            } else {
                lookup_table[i][j] = atan2(y, x);
            }
        }
    }
}

