#include "py/mphal.h"

static bool sram_valid = false;

bool sram_is_valid(void)
{
    return sram_valid;
}


void MAGELLAN_board_early_init(void) {
	#if MICROPY_HW_LCD43M
	mp_hal_pin_config(MICROPY_HW_MCULCD_CS, MP_HAL_PIN_MODE_OUTPUT, MP_HAL_PIN_PULL_UP, 0);
	mp_hal_pin_high(MICROPY_HW_MCULCD_CS); 
	#endif

	#if defined(MICROPY_HW_SRAM_SIZE)
	sram_init();
	sram_valid = true;
	UNUSED(sram_valid); 
	#if MICROPY_HW_SRAM_STARTUP_TEST
	sram_valid = sram_test(false);
	#endif
	#endif
}
