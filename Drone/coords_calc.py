import math
import numpy as np

pos = tuple[float, float, float]
""" Position vector represented as 3 floats being x, y, and z in that order.
"""

# default anchor positions in meters
POS_A1 = (-0.5,  0.5, 1.0)  # North-West
POS_A2 = ( 0.5,  0.5, 0.0)  # North-East
POS_A3 = (-0.5, -0.5, 0.0)  # South-West
POS_A4 = ( 0.5, -0.5, 0.0)  # South-East

def get_coords(
    d1: float, d2: float, d3: float, d4: float,
    pos_a1: pos = POS_A1, pos_a2: pos = POS_A2, pos_a3: pos = POS_A3,
    pos_a4: pos = POS_A4
) -> pos:
    """ Get coordinates of drone in local coordinate frame given the four
        distances returned by the UWB anchors.

        *NOTE: The enumeration of the anchors in the arguments descriptions may
        be subject to change!*

        Args:
            d1 (float): Distance to anchor 1, the North-West anchor in meters.
            d2 (float): Distance to anchor 2, the North-East anchor in meters.
            d3 (float): Distance to anchor 3, the South-West anchor in meters.
            d4 (float): Distance to anchor 4, the South-East anchor in meters.
            pos_a1 (pos, Optional): Position of anchor 1. Defaults to POS_A1.
            pos_a2 (pos, Optional): Position of anchor 2. Defaults to POS_A2.
            pos_a3 (pos, Optional): Position of anchor 3. Defaults to POS_A3.
            pos_a4 (pos, Optional): Position of anchor 4. Defaults to POS_A4.
        
        Returns:
            pos: The position as a tuple of 3 floats.
    """
    a = [
        [-2*pos_a1[0]+2*pos_a2[0],
         -2*pos_a1[1]+2*pos_a2[1],
         -2*pos_a1[1]+2*pos_a2[1]],
        
        [-2*pos_a2[0]+2*pos_a3[0],
         -2*pos_a2[1]+2*pos_a3[1],
         -2*pos_a2[1]+2*pos_a3[1]],
        
        [-2*pos_a3[0]+2*pos_a4[0],
         -2*pos_a3[1]+2*pos_a4[1],
         -2*pos_a3[1]+2*pos_a4[1]],
        
        [-2*pos_a1[0]+2*pos_a3[0],
         -2*pos_a1[1]+2*pos_a3[1],
         -2*pos_a1[1]+2*pos_a3[1]],
        
        [-2*pos_a1[0]+2*pos_a4[0],
         -2*pos_a1[1]+2*pos_a4[1],
         -2*pos_a1[1]+2*pos_a4[1]],
        
        [-2*pos_a2[0]+2*pos_a4[0],
         -2*pos_a2[1]+2*pos_a4[1],
         -2*pos_a2[1]+2*pos_a4[1]],
    ]

    b = [
        d1 ** 2 - d2 ** 2,
        d2 ** 2 - d3 ** 2,
        d3 ** 2 - d4 ** 2,
        d1 ** 2 - d3 ** 2,
        d1 ** 2 - d4 ** 2,
        d2 ** 2 - d4 ** 2
    ]

    return np.linalg.lstsq(a, b)

def generate_distances(
    true_pos: pos, noise: float = None,
    pos_a1: pos = POS_A1, pos_a2: pos = POS_A2, pos_a3: pos = POS_A3,
    pos_a4: pos = POS_A4
    ) -> tuple[float, float, float, float]:
    """ Generates test data given the true positions and can add noise to the
        data.

        Args:
            true_pos (pos): Intended position of the drone.
            noise (float, Optional): Standard deviation of bell curve random
                noise is generated from. For None no noise is applied. Defaults
                to None.
            pos_a1 (pos, Optional): Position of anchor 1. Defaults to POS_A1.
            pos_a2 (pos, Optional): Position of anchor 2. Defaults to POS_A2.
            pos_a3 (pos, Optional): Position of anchor 3. Defaults to POS_A3.
            pos_a4 (pos, Optional): Position of anchor 4. Defaults to POS_A4.
        
        Returns:
            tuple[float,float,float,float]: d1 to d4 as floats in a tuple.
    """
    if noise is not None:
        return (
            math.dist(true_pos, pos_a1) + np.random.normal(0.0, noise),
            math.dist(true_pos, pos_a2) + np.random.normal(0.0, noise),
            math.dist(true_pos, pos_a3) + np.random.normal(0.0, noise),
            math.dist(true_pos, pos_a4) + np.random.normal(0.0, noise)
        )
    else:
        return (
            math.dist(true_pos, pos_a1),
            math.dist(true_pos, pos_a2),
            math.dist(true_pos, pos_a3),
            math.dist(true_pos, pos_a4)
        )


if __name__ == "__main__":
    distances = generate_distances((0, 1, 1))
    print(distances)

    loesung = get_coords(
        *distances
    )
    print(loesung)