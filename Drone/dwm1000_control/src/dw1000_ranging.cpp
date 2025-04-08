#include "../inc/dwm1000_ranging.hpp"

DW1000Time DWMRanging::calculateToF() {
    /* Asymmetric two-sided two-way ranging */
    /* ToF = (T_round1 * T_round2 - T_reply1 * T_reply2) / (T_round1 + T_round2 + T_reply1 + T_reply2) */
}