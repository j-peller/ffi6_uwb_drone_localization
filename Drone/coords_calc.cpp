#include <iostream>
#include <chrono>
#include <random>
#include "eigen-3.4.0/Eigen/Dense"

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

/**
 * @brief Euclidean distance between to positions using pos datatype.
 * 
 * @param pos1 Point 1 in local coordinate frame.
 * @param pos2 Point 2 in local coordinate frame.
 * @return Distance between the 2 points.
 */
inline double euclidean_dist_pos(pos pos1, pos pos2) {
    return sqrt(
        (pos1.x - pos2.x) * (pos1.x - pos2.x)
        + (pos1.y - pos2.y) * (pos1.y - pos2.y)
        + (pos1.z - pos2.z) * (pos1.z - pos2.z)
    );
}

/**
 * @brief Get z value by resubstitution in distance equation.
 * 
 * @param d Distance to anchor.
 * @param x Calculated x of drone.
 * @param y Calculated y of drone.
 * @param pos_A Position of Anchor in local coordinate frame.
 * @return Calculated z of drone.
 */
double resubstitution_for_height(double d, double x, double y, pos pos_A) {
    return sqrt(
        d * d
        - (pos_A.x - x) * (pos_A.x - x) 
        - (pos_A.y - y) * (pos_A.y - y)
    ) - pos_A.z;
}

/**
 * @brief Calculates coodinates of the drone in the local coordinate frame.
 * 
 * @param d1 Distance of drone to Anchor 1 in local coordinate frame.
 * @param d2 Distance of drone to Anchor 2 in local coordinate frame.
 * @param d3 Distance of drone to Anchor 3 in local coordinate frame.
 * @param d4 Distance of drone to Anchor 4 in local coordinate frame.
 * @param pos_A1 Position of Anchor 1.
 * @param pos_A2 Position of Anchor 2.
 * @param pos_A3 Position of Anchor 3.
 * @param pos_A4 Position of Anchor 4.
 * @return Calculated position of drone in local coordinate frame.
 */
pos coords_calc(
    double d1, double d2, double d3, double d4,
    pos pos_A1, pos pos_A2, pos pos_A3, pos pos_A4
) {
    Eigen::Matrix<double, 3, 2> A;
    A << -2 * pos_A1.x + 2 * pos_A2.x, -2 * pos_A1.y + 2 * pos_A2.y,
        -2 * pos_A2.x + 2 * pos_A3.x, -2 * pos_A2.y + 2 * pos_A3.y,
        -2 * pos_A3.x + 2 * pos_A4.x, -2 * pos_A3.y + 2 * pos_A4.y;
    
    Eigen::Vector<double, 3> b;
    b << d1 * d1 - d2 * d2,
        d2 * d2 - d3 * d3,
        d3 * d3 - d4 * d4;
    
    Eigen::VectorXd solution = A.colPivHouseholderQr().solve(b);

    std::cout << solution << std::endl;

    double x = solution[0];
    double y = solution[1];
    double z = (resubstitution_for_height(d1, x, y, pos_A1)
        + resubstitution_for_height(d2, x, y, pos_A2)
        + resubstitution_for_height(d3, x, y, pos_A3)
        + resubstitution_for_height(d4, x, y, pos_A4)) / 4;

    return (pos) {x, y, z};
}

/**
 * @brief Generate UWB distance values with noise as test data.
 * 
 * @param true_pos True or known good position of the drone.
 * @param pos_A1 Position of Anchor 1 in local coordinate frame.
 * @param pos_A2 Position of Anchor 2 in local coordinate frame.
 * @param pos_A3 Position of Anchor 3 in local coordinate frame.
 * @param pos_A4 Position of Anchor 4 in local coordinate frame.
 * @param distance_uncertainty 95% confidence interval distance for UWB
 *  measurement in meters.
 * @return Distances d1 through d4 in a struct as doubles.
 */
distances generate_distances(
    pos true_pos, pos pos_A1, pos pos_A2, pos pos_A3, pos pos_A4,
    double distance_uncertainty
) {
    static unsigned seed = std::chrono::system_clock::now()
        .time_since_epoch().count();
    static std::default_random_engine generator(seed);
    static std::normal_distribution<double> distribution(
        0.0, distance_uncertainty / 1.96  // 95% confidence is at 1.96 sigma
    );
    
    return (distances) {
        euclidean_dist_pos(true_pos, pos_A1) + distribution(generator),
        euclidean_dist_pos(true_pos, pos_A2) + distribution(generator),
        euclidean_dist_pos(true_pos, pos_A3) + distribution(generator),
        euclidean_dist_pos(true_pos, pos_A4) + distribution(generator)
    };
}

/**
 * @brief Generate UWB distance values as test data.
 * 
 * @param true_pos True or known good position of the drone.
 * @param pos_A1 Position of Anchor 1 in local coordinate frame.
 * @param pos_A2 Position of Anchor 2 in local coordinate frame.
 * @param pos_A3 Position of Anchor 3 in local coordinate frame.
 * @param pos_A4 Position of Anchor 4 in local coordinate frame.
 * @return Distances d1 through d4 in a struct as doubles.
 */
distances generate_distances(
    pos true_pos, pos pos_A1, pos pos_A2, pos pos_A3, pos pos_A4
) {
    return (distances) {
        euclidean_dist_pos(true_pos, pos_A1),
        euclidean_dist_pos(true_pos, pos_A2),
        euclidean_dist_pos(true_pos, pos_A3),
        euclidean_dist_pos(true_pos, pos_A4)
    };
}

int main() {
    pos known_good = {15, 32, 10};

    pos A1 = {-0.5,  0.5, 0.0};
    pos A2 = { 0.5,  0.5, 0.0};
    pos A3 = {-0.5, -0.5, 0.0};
    pos A4 = { 0.5, -0.5, 0.0};
    
    distances dists = generate_distances(known_good, A1, A2, A3, A4, 0.1);

    pos ret_val = coords_calc(
        dists.d1, dists.d2, dists.d3, dists.d4,
        A1, A2, A3, A4
    );

    std::cout << "Position: " << ret_val.x << " " << ret_val.y << " " << ret_val.z << std::endl;
}