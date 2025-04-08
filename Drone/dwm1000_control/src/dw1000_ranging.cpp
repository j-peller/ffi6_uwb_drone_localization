#include "../inc/dwm1000_ranging.hpp"

void DWMRanging::calculateToF(DWM1000Device* anchor, DW1000Time* tof)
{
    /* Asymmetric two-sided two-way ranging */
    /* ToF = (T_round1 * T_round2 - T_reply1 * T_reply2) / (T_round1 + T_round2 + T_reply1 + T_reply2) */
}