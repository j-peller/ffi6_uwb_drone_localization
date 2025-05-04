#pragma once

#ifndef DW1000_MODES_H
#define DW1000_MODES_H

#include "dwm1000_ctrl.hpp"

typedef enum {
    THOTRO,
    JOPEL
} dw1000_mode_enum_t;

// Deklaration der konstanten Konfiguration
extern const dw1000_mode_t THOTRO110;
extern const dw1000_mode_t JOPEL110;


#endif