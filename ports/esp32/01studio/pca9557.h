#ifndef MICROPY_INCLUDED_PCA9557_H
#define MICROPY_INCLUDED_PCA9557_H
#include "hal_i2c.h"
#include "py/obj.h"

#define PCA9557_ADDR 0x19
#define SET_BITS(_m, _s, _v)  ((_v) ? (_m)|((_s)) : (_m)&~((_s)))

extern i2c_master_dev_handle_t pca9557_handle;

extern const mp_obj_type_t pyPod_pca9557_type;

void pca9557_init();
void pca9557_set_output_state(uint8_t pin, uint8_t level);
void lcd_cs(uint8_t level);
void pa_en(uint8_t level);
void dvp_pwdn(uint8_t level);
void pca9557_deinit();

#endif