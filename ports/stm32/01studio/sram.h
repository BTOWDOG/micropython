/*
 *
 * SRAM Driver.
 *
 */
#ifndef __SRAM_H__
#define __SRAM_H__

	#if defined(MICROPY_HW_SRAM_SIZE)
		#include <stdbool.h>
		
		bool sram_init(void);
		void *sram_start(void);
		void *sram_end(void);

		#if MICROPY_HW_SRAM_STARTUP_TEST
		bool sram_test(bool fast);
		#endif

		bool sram_is_valid(void);
	#endif
	
#endif // __SDRAM_H__
