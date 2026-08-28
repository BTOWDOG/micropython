
/********************************************************************************
	* Copyright (C), 2026 -2027, 01studio Tech. Co., Ltd.https://www.01studio.cc/
	* File Name				:	modgc308.c
	* Author				:	Folktale
	* Version				:	v1.0
	* date					:	2026/1/30
	* Description			:	
******************************************************************************/

#include <stdio.h>
#include <stdbool.h>
#include "py/runtime.h"
#include "py/obj.h"
#include "py/mphal.h"

#include "mpconfigboard.h"
#if MICROPY_HW_GC0308

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task.h"

#include "esp_camera.h"
#include "sensor.h"
#include "img_converters.h"
#include "global.h" 
#include "modgc0308.h"

#if MICROPY_ENABLE_STREAM
#include "http_stream.h"
#endif

#if MICROPY_HW_LCD15
#include "ST7789.h"
#endif

#if MICROPY_ENABLE_TFTLCD
#include "modtftlcd.h"
#include "lcd_spibus.h"
#endif


#if MICROPY_ENABLE_PCA9557
#include "hal_i2c.h"
#include "pca9557.h"
#endif


#define GC_TASK_PRIORITY    (ESP_TASK_PRIO_MIN + 1)
#define GC_TASK_STACK_SIZE  (16 * 1024)

typedef struct _gc308_obj_t
{
    mp_obj_base_t base;
} gc308_obj_t;

static framesize_t framesize;
static camera_fb_t *pic = NULL;
TaskHandle_t gc308_handle = NULL;

static bool is_display = false;
static bool is_snapshot = false;
static bool is_runtime = false;

#if MICROPY_ENABLE_TFTLCD
static bool is_init = false;
#endif

static uint16_t fb_buf = 2;
static camera_config_t camera_config={0};
//------------------------------------------------------------
static esp_err_t init_camera( pixformat_t pixel_format, framesize_t frame_size)
{
		#if MICROPY_ENABLE_PCA9557
		hal_i2c_init();
		pca9557_init();
		#endif
		dvp_pwdn(0);
		#if CAM_PIN_RESET
		camera_config.pin_reset  = CAM_PIN_RESET;
		#else
		camera_config.pin_reset  = -1;
		#endif
		camera_config.pin_pwdn = -1;
		camera_config.pin_xclk = CAMERA_PIN_XCLK;
		camera_config.pin_sscb_sda = CAMERA_PIN_SIOD;
		camera_config.pin_sscb_scl = CAMERA_PIN_SIOC;
		// camera_config.sccb_i2c_port = 0;

		camera_config.pin_d7 = CAMERA_PIN_D7;
		camera_config.pin_d6 = CAMERA_PIN_D6;
		camera_config.pin_d5 = CAMERA_PIN_D5;
		camera_config.pin_d4 = CAMERA_PIN_D4;
		camera_config.pin_d3 = CAMERA_PIN_D3;
		camera_config.pin_d2 = CAMERA_PIN_D2;
		camera_config.pin_d1 = CAMERA_PIN_D1;
		camera_config.pin_d0 = CAMERA_PIN_D0;
		camera_config.pin_vsync = CAMERA_PIN_VSYNC;
		camera_config.pin_href = CAMERA_PIN_HREF;
		camera_config.pin_pclk = CAMERA_PIN_PCLK;

		camera_config.xclk_freq_hz = XCLK_FREQ_HZ;
		camera_config.ledc_timer = LEDC_TIMER_1; 
		camera_config.ledc_channel = LEDC_CHANNEL_1; 
		camera_config.pixel_format = pixel_format; //YUV422,GRAYSCALE,RGB565,JPEG
		camera_config.frame_size = frame_size;    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG
		camera_config.jpeg_quality = 12; //0-63 12lower number means higher quality
		camera_config.fb_count = fb_buf;       //if more than one, i2s runs in continuous mode. Use only with JPEG
		camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

	//initialize the camera
	esp_err_t ret = esp_camera_init(&camera_config);
	if (ret != ESP_OK) {
		esp_camera_deinit();
		mp_raise_ValueError(MP_ERROR_TEXT("camera init Failed"));
	}
	sensor_t *s = esp_camera_sensor_get();
	s->set_hmirror(s, 0);

	return ret;
}

//======================================================================================================================
static mp_obj_t sensor_gc308_reset(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    return mp_obj_new_int(0);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(sensor_gc308_reset_obj, 0, sensor_gc308_reset);

//======================================================================================================================
static mp_obj_t sensor_gc308_setframesize(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{           
    #if MICROPY_ENABLE_PCA9557
    printf("warn resizing the framesize is not supported\n");
    #else
    static const mp_arg_t gc308_args [] = {
        { MP_QSTR_framesize,    MP_ARG_REQUIRED | MP_ARG_INT,   {.uint = FRAMESIZE_QQVGA} },
    };


    mp_arg_val_t args[MP_ARRAY_SIZE(gc308_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(gc308_args), gc308_args, args);
    
    if(args[0].u_int >= FRAMESIZE_QQQVGA && args[0].u_int <= FRAMESIZE_XGA){
        framesize =args[0].uint;
    }else{
        mp_raise_ValueError(MP_ERROR_TEXT("set framesize error"));
    }

    esp_camera_deinit();
    camera_config.frame_size = framesize;
    esp_err_t ret = esp_camera_init(&camera_config);
    if(ret != ESP_OK)
    {
        esp_camera_deinit();
        mp_raise_ValueError(MP_ERROR_TEXT("set framesize error"));
    }

    #if MICROPY_ENABLE_TFTLCD
    if(is_init)
    {
        grap_drawFill(0, 0, lcddev.width, lcddev.height, lcddev.backcolor);
    }
    #endif
	return mp_obj_new_int(framesize);
	#endif
	return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(sensor_gc308_setframesize_obj, 1, sensor_gc308_setframesize);
//-----------------------------------------------------------------------------------------------------------------------
static mp_obj_t sensor_gc308_hmirror(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t hmirror_args[] = {
        { MP_QSTR_value, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
    };
    
    mp_arg_val_t args[MP_ARRAY_SIZE(hmirror_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(hmirror_args), hmirror_args, args);

    uint8_t direction = args[0].u_int;
	
	if(direction != 0 && direction != 1)
	{
		mp_raise_ValueError(MP_ERROR_TEXT("hmirror must be 0 or 1"));
	}

    sensor_t *s = esp_camera_sensor_get();
    if(!s)
    {
        mp_raise_ValueError(MP_ERROR_TEXT("set hmirror Failed"));
    }
    s->set_hmirror(s, direction);

    return mp_obj_new_int(direction);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(sensor_gc308_hmirror_obj, 0, sensor_gc308_hmirror);

//----------------------------------------------------------------------------------------------------------------------
static mp_obj_t sensor_gc308_vflip(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t vflip_args[] = {
        { MP_QSTR_value, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(vflip_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(vflip_args), vflip_args, args);

    uint8_t direction = args[0].u_int;
	if(direction != 0 && direction != 1)
	{
		mp_raise_ValueError(MP_ERROR_TEXT("hmirror must be 0 or 1"));
	}

    sensor_t *s = esp_camera_sensor_get();
    if(!s)
    {
        mp_raise_ValueError(MP_ERROR_TEXT("set vflip Failed"));
    }
    s->set_vflip(s, direction);

    return mp_obj_new_int(direction);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(sensor_gc308_vflip_obj, 0, sensor_gc308_vflip);

//----------------------------------------------------------------------------------------------------------------------
static mp_obj_t sensor_gc308_snapshot(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
  	static const mp_arg_t snapshot_args[] = {
    	{ MP_QSTR_filepath,	MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
	};

	mp_arg_val_t args[MP_ARRAY_SIZE(snapshot_args)];
	mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(snapshot_args), snapshot_args, args);

	if(args[0].u_obj != MP_OBJ_NULL)
	{
		mp_buffer_info_t bufinfo;
		if(mp_obj_is_int(args[0].u_obj)){
			mp_raise_ValueError(MP_ERROR_TEXT("snapshot text parameter error"));
		}else{
			mp_get_buffer_raise(args[0].u_obj, &bufinfo, MP_BUFFER_READ);
			char *filename = bufinfo.buf;
			is_snapshot = true;
			mp_hal_delay_ms(10);

			uint32_t jpg_buf_len = 200*1024;
			size_t outsize = 0;
			uint8_t *outbuffer = (uint8_t *)heap_caps_malloc(jpg_buf_len, MALLOC_CAP_8BIT);
			if (outbuffer == NULL)
			{
				mp_raise_ValueError(MP_ERROR_TEXT("malloc outbuffer error"));
			}
			
			for(uint16_t i = 0; i < 2; i++)
			{
				pic = esp_camera_fb_get();
				if(i == 1 && pic != NULL)
				{
					if(rgb565_2jpg(pic, 50, jpg_buf_len, outbuffer, &outsize))
					{
						ssize_t w_res = jpg_save(filename, outbuffer, outsize);
						if(w_res != outsize)
						{
							printf("jpg save error:%d\r\n", w_res);
						}
					}

				}
				esp_camera_fb_return(pic);
				vTaskDelay(10 / portTICK_RATE_MS);
			}
			heap_caps_free(outbuffer);
			is_snapshot = false;

			return mp_obj_new_str(filename, strlen(filename));
		}
	}else{
		mp_raise_ValueError(MP_ERROR_TEXT("snapshot text parameter is empty"));
	}

	return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(sensor_gc308_snapshot_obj, 1, sensor_gc308_snapshot);

//----------------------------------------------------------------------------------------------------------------------
static void display_task(void *pvParameter)
{
    static uint16_t x = 0, y = 0;
    while(is_display)
    {
        if(!is_snapshot){
            pic = esp_camera_fb_get();
            if(pic)
            {
                x = (lcddev.width - pic->width)>>1;
                y = (lcddev.height - pic->height)>>1;
                grap_drawCam(x, y, pic->width, pic->height, (uint16_t *)pic->buf);
				esp_camera_fb_return(pic);
            }
        }else{
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }

    vTaskDelete(NULL);
}   

static mp_obj_t sensor_gc308_display(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    if(!is_init)
    {
        lcddev.backcolor = 0x0000;

        grap_drawFill(0, 0, lcddev.width, lcddev.height, lcddev.backcolor);
        lcddev.clercolor = lcddev.backcolor;
        is_init = true;
    }

	is_display = true;
	xTaskCreate(display_task, "display_task", GC_TASK_STACK_SIZE, NULL, GC_TASK_PRIORITY, &gc308_handle);

    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(sensor_gc308_display_obj, 0, sensor_gc308_display);

//----------------------------------------------------------------------------------------------------------------------
static mp_obj_t sensor_gc308_display_stop(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    is_display = false;
	mp_hal_delay_ms(100);
    grap_drawFill(0, 0, lcddev.width, lcddev.height, 0x0000);
    return mp_obj_new_int(is_display);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(sensor_gc308_display_stop_obj, 0, sensor_gc308_display_stop);

//-----------------------------------------------------------------------------------------------------------------------
#if MICROPY_ENABLE_STREAM
static mp_obj_t sensor_gc308_stream(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
	if(framesize >= FRAMESIZE_VGA){
		mp_raise_ValueError(MP_ERROR_TEXT("Camera framesize !> QVGA"));
	}
	init_httpd_app(80, 0);

	return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(sensor_gc308_stream_obj, 0, sensor_gc308_stream);
#endif

//-----------------------------------------------------------------------------------------------------------------------
static mp_obj_t sensor_gc308_deinit(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    is_display = false;
	esp_err_t ret = esp_camera_deinit();
    if(ret != ESP_OK){
        mp_raise_ValueError(MP_ERROR_TEXT("Camera deinit Failed"));
        return mp_const_false;
    }
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(sensor_gc308_deinit_obj, 0, sensor_gc308_deinit);

void gc0308_deinit()
{
    if(is_runtime)
    {
        is_display = false;
		is_runtime = false;

        esp_camera_deinit();
    }
}
//-----------------------------------------------------------------------------------------------------------------------
static mp_obj_t sensor_gc308_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {

	static const mp_arg_t allowed_args[] = {
		{ MP_QSTR_frame, MP_ARG_INT, {.u_int = 1} },
	};

	mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
	mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
	fb_buf = args[0].u_int;
	
	if(fb_buf >= 2) fb_buf = 2;
	else fb_buf = 1;

	framesize = FRAMESIZE_240X240;
	init_camera(PIXFORMAT_RGB565, framesize);
	is_runtime = true;
	gc308_obj_t *gc308_obj;
	gc308_obj = m_new_obj(gc308_obj_t);
	gc308_obj->base.type = &sensor_gc308_type;
	return MP_OBJ_FROM_PTR(gc308_obj);
}

//--------------------------------------------------------------------------------------------------------------
static const mp_rom_map_elem_t sensor_gc308_locals_dict_table[]  = {
    { MP_ROM_QSTR(MP_QSTR__NAME__), MP_ROM_QSTR(MP_QSTR_gc308) },
	{ MP_ROM_QSTR(MP_QSTR_set_framesize), MP_ROM_PTR(&sensor_gc308_setframesize_obj) },
	{ MP_ROM_QSTR(MP_QSTR_set_hmirror), MP_ROM_PTR(&sensor_gc308_hmirror_obj) },
	{ MP_ROM_QSTR(MP_QSTR_set_vflip), MP_ROM_PTR(&sensor_gc308_vflip_obj) },
	{ MP_ROM_QSTR(MP_QSTR_snapshot), MP_ROM_PTR(&sensor_gc308_snapshot_obj) },
	{ MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&sensor_gc308_reset_obj) },
	{ MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&sensor_gc308_deinit_obj) },
	#if MICROPY_ENABLE_STREAM
	{ MP_ROM_QSTR(MP_QSTR_stream), MP_ROM_PTR(&sensor_gc308_stream_obj) },
	#endif
	#if MICROPY_ENABLE_TFTLCD
	{ MP_ROM_QSTR(MP_QSTR_display), MP_ROM_PTR(&sensor_gc308_display_obj) },
	{ MP_ROM_QSTR(MP_QSTR_display_stop), MP_ROM_PTR(&sensor_gc308_display_stop_obj) },
	#endif
};
static MP_DEFINE_CONST_DICT(sensor_gc308_locals_dict, sensor_gc308_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    sensor_gc308_type,
    MP_QSTR_GC0308,
    MP_TYPE_FLAG_NONE,
    make_new, sensor_gc308_make_new,
    locals_dict, &sensor_gc308_locals_dict
);

#endif