#ifndef COORDS_CALC_H
#define COORDS_CALC_H

// for Windows compilation
// #include "eigen-3.4.0/Eigen/Dense"

// for Linux compilation
#include  <Eigen/Dense>

// structs
typedef struct {
    double x;
    double y;
    double z;
} pos;

typedef struct {
    double d1;
    double d2;
    double d3;
    double d4;
} distances;

typedef distances distances_t;


// function headers
inline double euclidean_dist_pos(pos pos1, pos pos2);
pos coords_calc(
    double d1, double d2, double d3, double d4,
    pos pos_A1, pos pos_A2, pos pos_A3, pos pos_A4
);
distances generate_distances(
    pos true_pos, pos pos_A1, pos pos_A2, pos pos_A3, pos pos_A4,
    double distance_uncertainty
);
distances generate_distances(
    pos true_pos, pos pos_A1, pos pos_A2, pos pos_A3, pos pos_A4
);

#endif
