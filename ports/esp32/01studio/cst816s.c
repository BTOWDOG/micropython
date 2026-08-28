#include "mpconfigboard.h"
#include "freertos/FreeRTOS.h"
#include "py/runtime.h"

#if (MICROPY_HW_CST816S & MICROPY_ENABLE_TOUCH)
#include "hal_i2c.h"

#include "modtftlcd.h"
#include "modtouch.h"
#include "ST7789.h"


#include "cst816s.h"

i2c_master_dev_handle_t cst816s;

typedef struct _touch_cst816s_obj_t
{
    mp_obj_base_t base;
}touch_cst816s_obj_t;

static uint16_t touch_w,touch_h;

static touch_cst816s_obj_t cst816s_obj;

//--------------------------------------------------------------------------------------------------------
static esp_err_t cst816s_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(
        cst816s,
        &reg,
        1,
        data,
        len,
        -1
    );
}

//-------------------------------------------------------------------------------------------------------
void cst816s_read_point()
{

    uint16_t input_x = 0;
    uint16_t input_y = 0;

	uint8_t read[6];
    uint8_t status;
    uint8_t touch_num = 0;
    int8_t read_id = 0;
    static uint8_t pre_touch = 0; 

    cst816s_read_regs(0x01, read, 6);
    status = read[1];
    touch_num = 1;
    
    if(pre_touch > touch_num)
    {
        tp_touch_up(read_id);
    }

    if(status){
		switch (tp_dev.dir)
		{
		case 2:
			input_y = touch_h - (((read[2] & 0x0F) << 8) | read[3]);
			input_x = ((read[4] & 0x0F) << 8) | read[5];

			break;
		case 3:
			input_x = touch_w - (((read[2] & 0x0F) << 8) | read[3]);
			input_y = touch_h - (((read[4] & 0x0F) << 8) | read[5]);
			break;
		case 4:
			input_y = (((read[2] & 0x0F) << 8) | read[3]);
			input_x = touch_w -(((read[4] & 0x0F) << 8) | read[5]);
			break;
		default:
			input_x = ((read[2] & 0x0F) << 8) | read[3];
			input_y = ((read[4] & 0x0F) << 8) | read[5];
			break;
		}
        
		if(input_x > touch_w || input_y > touch_h){
			return;
		}
		
		tp_touch_down(read_id, input_x, input_y, 0);
    }else if(pre_touch){
        tp_touch_up(read_id);
    }
    pre_touch = touch_num;
}

//-----------------------------------------------------------------------------------------------------------------------------
static mp_obj_t touch_cst816s_read(void)
{
    mp_obj_t tuple[3];
    cst816s_read_point();
    if (tp_dev.sta&TP_PRES_DOWN){
        tuple[0] = mp_obj_new_int(0);
    }else if(tp_dev.sta&TP_PRES_MOVE){	
        tuple[0] = mp_obj_new_int(1); 
    }else{ 	
        tuple[0] = mp_obj_new_int(2);
    }

	tuple[1] = mp_obj_new_int(tp_dev.x[0]);
	tuple[2] = mp_obj_new_int(tp_dev.y[0]);
	return mp_obj_new_tuple(3, tuple);
}
static MP_DEFINE_CONST_FUN_OBJ_0(touch_cst816s_read_obj, touch_cst816s_read);

//----------------------------------------------------------------------------------------------------------------------------
static mp_obj_t touch_cst816s_scan(void)
{
    cst816s_read_point();
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_0(touch_cst816s_scan_obj, touch_cst816s_scan);

//----------------------------------------------------------------------------------------------------------------------------
static mp_obj_t cst816s_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{
	static const mp_arg_t allowed_args[] = {
			{ MP_QSTR_portrait, MP_ARG_INT, {.u_int = 1} },
	};
	
	mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
	mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    if(args[0].u_int != 0)
    {
        tp_dev.dir = args[0].u_int;
    }

    switch (tp_dev.dir)
    {
        default:
            touch_w = 240;
            touch_h = 240;
            break;
    }

    cst816s = i2c_add_dev(0x15);

    cst816s_obj.base.type = &touch_cst816s_type;
    return MP_OBJ_FROM_PTR(&touch_cst816s_type);
}
static const mp_rom_map_elem_t cst816s_locals_dict_table[] = {
	{ MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_cst816s) },
	{ MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&touch_cst816s_read_obj) },
	{ MP_ROM_QSTR(MP_QSTR_tick_inc), MP_ROM_PTR(&touch_cst816s_scan_obj) },

};
static MP_DEFINE_CONST_DICT(cst816s_locals_dict, cst816s_locals_dict_table);


MP_DEFINE_CONST_OBJ_TYPE(
    touch_cst816s_type,
    MP_QSTR_CST816S,
    MP_TYPE_FLAG_NONE,
    make_new, cst816s_make_new,
    locals_dict, &cst816s_locals_dict
);



#endif