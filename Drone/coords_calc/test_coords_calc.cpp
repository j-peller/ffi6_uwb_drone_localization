#include <iostream>
#include "coords_calc.h"

/**
 * @brief Test coords calc by giving x, y, z known goods as command line args.
 *  A normally distributed noise being within 10 cm in 95 % of cases is assumed.
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char* argv[]) {
    pos known_good = {
        std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3])
    };

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