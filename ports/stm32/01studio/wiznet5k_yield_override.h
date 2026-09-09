#ifndef MICROPY_INCLUDED_WIZNET5K_YIELD_OVERRIDE_H
#define MICROPY_INCLUDED_WIZNET5K_YIELD_OVERRIDE_H

#include <stdbool.h>

#include "wizchip_conf.h"

#undef WIZCHIP_YIELD

#define MPY_WIZNET_SOCKERR_INTR (-16)


void mpy_wiznet_yield(void);
bool mpy_wiznet_abort_requested(void);

#define WIZCHIP_YIELD()                         \
    do {                                        \
        mpy_wiznet_yield();                     \
        if (mpy_wiznet_abort_requested()) {     \
            return MPY_WIZNET_SOCKERR_INTR;     \
        }                                       \
    } while (0)

#endif
