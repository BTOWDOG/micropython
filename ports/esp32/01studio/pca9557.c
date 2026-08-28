#include "mpconfigboard.h"
#include "freertos/FreeRTOS.h"
#if MICROPY_ENABLE_PCA9557
#include "pca9557.h"

i2c_master_dev_handle_t pca9557_handle;
static bool is_init = false;

typedef struct _pyPod_pca9557_obj_t
{
    mp_obj_base_t base;
    mp_obj_t callback;
}pca9557_obj_t;


void pca9557_init()
{
    if(!is_init)
    {
        i2c_device_config_t i2c_dev_cfg ={
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = PCA9557_ADDR,
            .scl_speed_hz = 100 *1000,
        };
        i2c_master_bus_add_device(i2c_bus_, &i2c_dev_cfg, &pca9557_handle);
        i2c_write_reg(pca9557_handle, 0x01, 0x00);
        i2c_write_reg(pca9557_handle, 0x03, 0xf8);
        is_init = true;
    }

}

void pca9557_set_output_state(uint8_t pin, uint8_t level)
{
    uint8_t data;
    i2c_read_reg(pca9557_handle, 0x00, &data);
    data = SET_BITS(data, pin, level);
    i2c_write_reg(pca9557_handle, 0x01, data);
    
}

void lcd_cs(uint8_t level)
{
    pca9557_set_output_state(LCD_CS_PIN, level);
}

void dvp_pwdn(uint8_t level)
{
    pca9557_set_output_state(DVP_PWDN_PIN, level);
}

void pa_en(uint8_t level)
{
    pca9557_set_output_state(PA_EN_PIN, level);
}

void pca9557_deinit()
{
    // vTaskDelay(pdMS_TO_TICKS(50));
    // i2c_master_bus_rm_device(pca9557_handle);
    // is_init = false;
    // i2c_write_reg(pca9557_handle, 0x01, 0x01);
    // i2c_write_reg(pca9557_handle, 0x03, 0xf8);
}
//----------------------------------------------------------------------------------------------------------------------------------
static mp_obj_t pyPod_pca9557_read(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    uint8_t data;
    i2c_read_reg(pca9557_handle, 0x00, &data);
    
    return mp_obj_new_int_from_uint(data);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(pyPod_pca9557_read_obj, 0, pyPod_pca9557_read);
//----------------------------------------------------------------------------------------------------------------------------------
static mp_obj_t pyPod_pca9557_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{
    // mp_arg_check_num(n_args, n_kw, 0, MP_OBJ_FUN_ARGS_MAX, true);
    
    hal_i2c_init();
    pca9557_init();

    pca9557_obj_t *pca9557_obj;
    pca9557_obj = m_new_obj(pca9557_obj_t);
    pca9557_obj->base.type = &pyPod_pca9557_type;
    pca9557_obj->callback = mp_const_none;

    return MP_OBJ_FROM_PTR(pca9557_obj);

}

//-----------------------------------------------------------------------------------------------------------------------------------
static const mp_rom_map_elem_t pyPod_pca9557_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR__name__), MP_ROM_QSTR(MP_QSTR_pca9557) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&pyPod_pca9557_read_obj) },
};

static MP_DEFINE_CONST_DICT(pyPod_pca9557_locals_dict, pyPod_pca9557_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    pyPod_pca9557_type,
    MP_QSTR_PCA9557,
    MP_TYPE_FLAG_NONE,
    make_new, pyPod_pca9557_make_new,
    locals_dict, &pyPod_pca9557_locals_dict

);

#endif