#include "dwm1000_ctrl.hpp"
#include "dwm1000_ranging.hpp"

#include <stdio.h>

int main() {

    switch (ROLE) {
        case dwm1000_role_t::DRONE:
            break;
        case dwm1000_role_t::ANCHOR:
            break;
    }

    return EXIT_SUCCESS;
}