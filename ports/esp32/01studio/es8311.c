#include <stdio.h>
#include <stdbool.h>
#include "py/runtime.h"
#include "py/obj.h"
#include "py/mphal.h"

#include "lib/oofatfs/ff.h"

#include "mpconfigboard.h"
#if MICROPY_HW_ES8311

#if MICROPY_ENABLE_PCA9557
#include "pca9557.h"
#include "hal_i2c.h"
#endif

#include "esp_vfs_fat.h"
#include "hal_i2c.h"
#include "es8311.h"

#include "global.h"

#include "py/runtime.h"
#include "py/stream.h"
#include "py/objstr.h"
#include "esp_system.h"
#include "esp_audio_simple_dec_default.h"
#include "esp_audio_dec_default.h"
#include "esp_audio_dec_reg.h"
#include "esp_audio_simple_dec.h"
#include "esp_task.h"
#include "esp_timer.h"

#define AUDIO_TASK_PRIORITY        (ESP_TASK_PRIO_MIN + 1)
#define AUDIO_TASK_STACK_SIZE      (16 * 1024)

#define IN_BUF_LEN                 (4 * 1024)
#define OUT_BUF_LEN                (5 * 1024)

#define READ_SIZE       2048

typedef struct _audio_es8311_obj_t{
    mp_obj_base_t base;
    mp_obj_t callback;
}es8311_obj_t;

TaskHandle_t es8311_handle = NULL;
TaskHandle_t recorder_handle = NULL;
static esp_audio_simple_dec_type_t dec_type;

__audiodev audiodev;

static bool is_rec;
static bool is_init = false;
//-----------------------------------------------------------------------------------------------------------------------
void wav_header_init(wav_header_t *hdr, int sample_rate, int bits, int channels)
{
    memcpy(hdr->riff, "RIFF", 4);
    memcpy(hdr->wave, "WAVE", 4);
    memcpy(hdr->fmt,  "fmt ", 4);
    memcpy(hdr->data, "data", 4);

    hdr->fmt_size = 16;
    hdr->audio_format = 1;
    hdr->num_channels = channels;
    hdr->sample_rate = sample_rate;
    hdr->bits_per_sample = bits;
    hdr->block_align = channels * bits / 8;
    hdr->byte_rate = hdr->block_align * sample_rate;

    hdr->data_size = 0;
    hdr->file_size = 36;
}

void recorder_task()
{
    
    i2s_channel_enable(audiodev.tx_handle);
    i2s_channel_enable(audiodev.rx_handle);
    pa_en(0);
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel = 1, 
        .channel_mask = 0,
        .sample_rate = (uint32_t)16000,
        .mclk_multiple = 0,
    };
    esp_codec_dev_open(audiodev.dev, &fs);

    wav_header_t header;
    wav_header_init(&header, 16000, 16, 1);
    int errcode = 0;

    audiodev.stream->write(audiodev.file, &header, sizeof(header), &errcode);

    uint8_t *buffer = heap_caps_malloc(READ_SIZE, MALLOC_CAP_8BIT);

    size_t total_bytes = 0;

    is_rec = true;

    while (is_rec)
    {
        esp_err_t ret = esp_codec_dev_read(audiodev.dev, buffer, READ_SIZE);

        if(ret == ESP_OK){
            audiodev.stream->write(audiodev.file, buffer, READ_SIZE, &errcode);
            total_bytes += READ_SIZE;
        }
        
        vTaskDelay(1);
    }
    
    header.data_size = total_bytes;
    header.file_size = total_bytes + 36;

    struct mp_stream_seek_t seek = {
        .offset = 0,
        .whence = SEEK_SET,
    };
    audiodev.stream->ioctl(audiodev.file, MP_STREAM_SEEK,(uintptr_t)&seek, &errcode);
    audiodev.stream->write(audiodev.file, &header, sizeof(header), &errcode);

    free(buffer);
    
    if(audiodev.file != NULL){
        audiodev.stream->ioctl(audiodev.file, MP_STREAM_CLOSE, 0, &errcode);
        audiodev.file = MP_OBJ_NULL;
    }
    
    esp_codec_dev_close(audiodev.dev);
    
    vTaskDelete(NULL);
    
}

static void es8311_recorder(const char *name)
{

    mp_obj_t open_args[2] = {
        mp_obj_new_str(name, strlen(name)),
        mp_obj_new_str("wb", 2),
    };

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        audiodev.file = mp_vfs_open(2, open_args, (mp_map_t *)&mp_const_empty_map);
        nlr_pop();
        audiodev.stream = mp_get_stream_raise(audiodev.file, MP_STREAM_OP_WRITE);
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("failed to load file"));
    }

    xTaskCreate(recorder_task, "recorder_task", AUDIO_TASK_STACK_SIZE / sizeof(StackType_t), NULL, AUDIO_TASK_PRIORITY, &recorder_handle);
    
}

static mp_obj_t audio_es8311_record(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    static const mp_arg_t record_args[] = {
        { MP_QSTR_filename,    MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
        { MP_QSTR_db,     			 MP_ARG_INT, {.u_int = 50} }, //
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(record_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(record_args), record_args, args);

    
    //MP_OBJ_NULL
    if(args[0].u_obj !=MP_OBJ_NULL) {
        mp_buffer_info_t bufinfo;
        if (mp_obj_is_int(args[0].u_obj)) {
            mp_raise_ValueError(MP_ERROR_TEXT("input record file name error"));
        } 
	    else{
		    mp_get_buffer_raise(args[0].u_obj, &bufinfo, MP_BUFFER_READ);
		    char *str = bufinfo.buf;
            if(audiodev.state != AUDIO_STATE_IDLE){
                audiodev.state = AUDIO_STATE_IDLE;
                vTaskDelay(50);
            }
            esp_codec_dev_set_in_gain(audiodev.dev, (uint8_t)(args[1].u_int * 0.60));
		    es8311_recorder(str);
        }
    }else{
        mp_raise_ValueError(MP_ERROR_TEXT("record file name parameter is empty"));
    }
    
    return mp_const_none;
}

static MP_DEFINE_CONST_FUN_OBJ_KW(audio_es8311_record_obj, 1, audio_es8311_record);
//-----------------------------------------------------------------------------------------------------------------------
static mp_obj_t audio_es8311_record_stop()
{
    is_rec = false;
    vTaskDelay(10);
    
    return mp_obj_new_int(is_rec);
}

static MP_DEFINE_CONST_FUN_OBJ_KW(audio_es8311_record_stop_obj, 0, audio_es8311_record_stop);
//-----------------------------------------------------------------------------------------------------------------------
static mp_obj_t audio_es8311_load(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t es8311_allowed_args[] = {
        { MP_QSTR_filename, MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(es8311_allowed_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(es8311_allowed_args), es8311_allowed_args, args);

    if(args[0].u_obj != MP_OBJ_NULL){
        mp_buffer_info_t bufinfo; //官方不建议使用 缓存流  建议mp_obj_str_get_str(path_obj);
        if(mp_obj_is_int(args[0].u_obj)){
            mp_raise_ValueError(MP_ERROR_TEXT("wav name parameter error"));
        }else{
            mp_get_buffer_raise(args[0].u_obj, &bufinfo, MP_BUFFER_READ);
            memset(audiodev.file_path, '\0', 50);
            strncpy(audiodev.file_path, (char *)bufinfo.buf, bufinfo.len);
            audiodev.len = bufinfo.len;
            
            if(audiodev.state != AUDIO_STATE_IDLE){
                audiodev.state = AUDIO_STATE_IDLE;
                vTaskDelay(200);
            }

            mp_obj_t open_args[2] = {
                mp_obj_new_str(audiodev.file_path, audiodev.len),
                mp_obj_new_str("rb", 2),
            };

            nlr_buf_t nlr;
            if (nlr_push(&nlr) == 0) {
                audiodev.file = mp_vfs_open(2, open_args, (mp_map_t *)&mp_const_empty_map);
                nlr_pop();
                audiodev.stream = mp_get_stream_raise(audiodev.file, MP_STREAM_OP_READ);
            } else {

                mp_raise_ValueError(MP_ERROR_TEXT("failed to read file"));
            }
            
        }    
    }
    else{
        mp_raise_ValueError(MP_ERROR_TEXT("Load wav is empty"));
    }

    
    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(audio_es8311_load_obj, 1, audio_es8311_load);

//-------------------------------------------------------------------------------------

#define CLAMP16(x) ((x) > 32767 ? 32767 : ((x) < -32768 ? -32768 : (x)))

#define CLAMP16(x) ((x) > 32767 ? 32767 : ((x) < -32768 ? -32768 : (x)))

static inline int16_t stereo_to_mono_wm_like(int16_t L, int16_t R)
{
    int32_t m = (int32_t)L + (int32_t)R;

    // 平均
    m >>= 1;

    // 轻微补偿（非常接近 WM 芯片）
    m = (m * 6) / 5;   // ≈ 1.2x

    return CLAMP16(m);
}


void stereo_to_mono_inplace(int16_t *pcm, size_t frames)
{
    for (size_t i = 0; i < frames; i++) {
        int16_t L = pcm[2*i];
        int16_t R = pcm[2*i + 1];

        int16_t mono = stereo_to_mono_wm_like(L, R);

        pcm[2*i]     = mono;  // Left
        pcm[2*i + 1] = mono;  // Right
    }
}


void audio_play_task()
{
    esp_audio_dec_register_default();
    esp_audio_simple_dec_register_default();

    esp_audio_dec_handle_t dec = NULL;
    esp_audio_simple_dec_cfg_t dec_cfg = {
        .dec_type = dec_type,
    };

    if(esp_audio_simple_dec_open(&dec_cfg, &dec) != ESP_AUDIO_ERR_OK){
        printf("error\n");
    }

    uint8_t *in_buf  = malloc(IN_BUF_LEN);
    uint8_t *out_buf = malloc(OUT_BUF_LEN);
    if(!in_buf || !out_buf){
        printf("mem error");
    }

    esp_audio_simple_dec_raw_t raw;
    esp_audio_simple_dec_out_t out = {
        .buffer = out_buf,
        .len    = OUT_BUF_LEN,
    };
    
    size_t cached = 0;
    int errcode = 0;

    esp_audio_err_t ret = ESP_AUDIO_ERR_OK;
    mp_hal_stdout_tx_str("start audio play\n");

    while (1){
        
        if(audiodev.state == AUDIO_STATE_PLAY){
            if(cached < IN_BUF_LEN){
                size_t r = audiodev.stream->read(audiodev.file, in_buf + cached, IN_BUF_LEN - cached, &errcode);

                if(r == 0 && cached == 0){
                    pa_en(0);
                    audiodev.state = AUDIO_STATE_IDLE;
                }
                cached += r;
            }
            raw.buffer   = in_buf;
            raw.len      = cached;
            raw.consumed = 0;
            while (raw.len > 0){
                ret = esp_audio_simple_dec_process(dec, &raw, &out);
                if(ret == ESP_AUDIO_ERR_OK ){
                    if(out.decoded_size > 0){
                        if(!audiodev.input_enabled){
                            esp_audio_simple_dec_info_t info = {};
                            esp_audio_simple_dec_get_info(dec, &info);

                            i2s_channel_enable(audiodev.tx_handle);
                            i2s_channel_enable(audiodev.rx_handle);
                            
                            esp_codec_dev_sample_info_t fs = {
                                .sample_rate = info.sample_rate,
                                .channel     = info.channel,
                                .bits_per_sample = info.bits_per_sample,
                            };
                            pa_en(1);
                            esp_codec_dev_open(audiodev.dev, &fs);
                            audiodev.input_enabled = true ;
                            
                            // printf(
                            //     "sample_rate=%ld bits=%d ch=%d\n",
                            //     info.sample_rate,
                            //     info.bits_per_sample,
                            //     info.channel);
                        }
                        
                    }
                    size_t frames = out.len / (2 * sizeof(int16_t));
                    stereo_to_mono_inplace((int16_t *)out.buffer, frames);    
                    esp_codec_dev_write(audiodev.dev, out.buffer,  out.decoded_size);

                    raw.buffer += raw.consumed;
                    raw.len    -= raw.consumed;
                }else{
                    // printf("Fail to decode dat ret %d\n", ret);
                    mp_hal_stdout_tx_str("Fail to decode dat ret");
                    audiodev.state = AUDIO_STATE_IDLE;
                    break; 
                }
                // mp_handle_pending(true);
                // vTaskDelay(1);  
            }
            
            if(raw.len > 0){
                memmove(in_buf, raw.buffer, raw.len);
            }
            cached = raw.len;
            vTaskDelay(1);
        }else if(audiodev.state == AUDIO_STATE_PAUSE){
            vTaskDelay(50 / portTICK_RATE_MS);
        }else if(audiodev.state == AUDIO_STATE_STOP){
            struct mp_stream_seek_t seek = {
                .offset = 0,
                .whence = SEEK_SET,
            };
            audiodev.stream->ioctl(audiodev.file, MP_STREAM_SEEK,(uintptr_t)&seek, &errcode);
            audiodev.state = AUDIO_STATE_PAUSE;
            vTaskDelay(10 / portTICK_RATE_MS);
        }else if(audiodev.state == AUDIO_STATE_IDLE){
            vTaskDelay(10 / portTICK_RATE_MS);
            break;
        }
        // mp_handle_pending(true);
        // vTaskDelay(1);  
        
    }
    esp_codec_dev_close(audiodev.dev);
    audiodev.input_enabled = false; 
    
    esp_audio_simple_dec_close(dec);
    esp_audio_simple_dec_unregister_default();
    esp_audio_dec_unregister_default();

    if(in_buf != NULL){
        free(in_buf);
    }

    if(out_buf != NULL){
        free(out_buf);
    }

    if(audiodev.file != NULL){
        audiodev.stream->ioctl(audiodev.file, MP_STREAM_CLOSE, 0, &errcode);
        audiodev.file = MP_OBJ_NULL;

    }

    mp_hal_stdout_tx_str("audio play end\n");
    vTaskDelete(NULL);
}

static mp_obj_t audio_es8311_play(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    if(audiodev.state == AUDIO_STATE_IDLE){

        const char *type = mp_obj_str_get_str(file_type(audiodev.file_path));
        if(strncmp(type, "wav", 3) == 0 || strncmp(type, "WAV", 3) == 0){
            dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_WAV;
        }else if(strncmp(type, "mp3", 3) == 0 || strncmp(type, "mp3", 3) == 0){
            dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_MP3;
        }else{
            mp_raise_ValueError(MP_ERROR_TEXT("play audio type error"));
        }

        audiodev.state = AUDIO_STATE_PLAY;
        
        xTaskCreate(audio_play_task, "audio_play_task", AUDIO_TASK_STACK_SIZE / sizeof(StackType_t), NULL, AUDIO_TASK_PRIORITY , &es8311_handle);
        // audio_play_task();
    }


    return mp_const_true;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(audio_es8311_play_obj, 0, audio_es8311_play);
//----------------------------------------------------------------------------------------------------
static mp_obj_t audio_es8311_continue_play(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    if(audiodev.state == AUDIO_STATE_PAUSE){
        audiodev.state = AUDIO_STATE_PLAY;

        vTaskDelay(10);
        return mp_const_true;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(audio_es8311_continue_play_obj, 0, audio_es8311_continue_play);

//----------------------------------------------------------------------------------------------------
static mp_obj_t audio_es8311_pause(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    if(audiodev.state == AUDIO_STATE_PLAY){
        audiodev.state = AUDIO_STATE_PAUSE;
        vTaskDelay(20);
        esp_codec_dev_close(audiodev.dev);
        pa_en(0);
        audiodev.input_enabled = false;
        return mp_const_true;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(audio_es8311_pause_obj, 0, audio_es8311_pause);

//----------------------------------------------------------------------------------------------------
static mp_obj_t audio_es8311_stop(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    if(audiodev.state == AUDIO_STATE_PLAY || audiodev.state == AUDIO_STATE_PAUSE){
        audiodev.state = AUDIO_STATE_STOP;
        vTaskDelay(20);
        esp_codec_dev_close(audiodev.dev);
        pa_en(0);
        audiodev.input_enabled = false;
    
        return mp_const_true;
    }
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_KW(audio_es8311_stop_obj, 0, audio_es8311_stop);

//----------------------------------------------------------------------------------------------------
static mp_obj_t audio_es8311_volume(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args)
{
    static const mp_arg_t volume_args[] = {
        { MP_QSTR_vol, MP_ARG_INT, {.u_int = 80} },
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(volume_args)];
    mp_arg_parse_all(n_args-1, pos_args+1, kw_args, MP_ARRAY_SIZE(volume_args), volume_args, args);

    uint8_t vul = (uint8_t)(args[0].u_int);
    esp_codec_dev_set_out_vol(audiodev.dev, vul);
    
    return mp_obj_new_int(vul);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(audio_es8311_volume_obj, 0, audio_es8311_volume);

//----------------------------------------------------------------------------------------------------
static void audio_es8311_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind)
{
    mp_printf(print, "es8311 printf\n");
}
//-------------------------------------------------------------------------------------
void es8311_init()
{
    if(!is_init)
    {
        i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(BSP_I2S_NUM, I2S_ROLE_MASTER);
        i2s_new_channel(&chan_cfg, &audiodev.tx_handle, &audiodev.rx_handle);

        i2s_std_config_t std_cfg = {
            .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_OUTPUT_SAMPLE_RATE),
            .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(16, I2S_SLOT_MODE_STEREO), 
            .gpio_cfg = {
                .mclk = AUDIO_I2S_GPIO_MCLK,
                .bclk = AUDIO_I2S_GPIO_BCLK,
                .ws   = AUDIO_I2S_GPIO_WS,
                .dout = AUDIO_I2S_GPIO_DOUT,
                .din  = AUDIO_I2S_GPIO_DIN,
                .invert_flags = {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false
                }
            }
        };

        i2s_channel_init_std_mode(audiodev.tx_handle, &std_cfg);

        std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
        i2s_channel_init_std_mode(audiodev.rx_handle, &std_cfg);

        audio_codec_i2s_cfg_t i2s_cfg = {
            .port = BSP_I2S_NUM,
            .rx_handle = audiodev.rx_handle,
            .tx_handle = audiodev.tx_handle,
        };
        audiodev.data_if = audio_codec_new_i2s_data(&i2s_cfg); 

        audio_codec_i2c_cfg_t i2c_cfg = {
            .addr = ES8311_CODEC_DEFAULT_ADDR,
            .bus_handle = i2c_bus_,
        };
        audiodev.ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);

        audiodev.gpio_if = audio_codec_new_gpio();

        es8311_codec_cfg_t es8311_cfg = {};
        es8311_cfg.ctrl_if = audiodev.ctrl_if;
        es8311_cfg.gpio_if = audiodev.gpio_if;
        es8311_cfg.codec_mode = ESP_CODEC_DEV_WORK_MODE_DAC;
        es8311_cfg.pa_pin = GPIO_NUM_NC;
        es8311_cfg.use_mclk = true;
        es8311_cfg.hw_gain.pa_voltage = 5.0;
        es8311_cfg.hw_gain.codec_dac_voltage = 3.3;
        audiodev.codec_if = es8311_codec_new(&es8311_cfg);

        esp_codec_dev_cfg_t dev_cfg = {
            .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
            .codec_if = audiodev.codec_if,
            .data_if  = audiodev.data_if,
        };
        audiodev.dev = esp_codec_dev_new(&dev_cfg);

        esp_codec_dev_set_out_vol(audiodev.dev, 50);

        is_init = true;
    }
}
int i = 0;
static mp_obj_t audio_es8311_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args)
{
    mp_arg_check_num(n_args, n_kw, 0, MP_OBJ_FUN_ARGS_MAX, true);
    
    hal_i2c_init();
    pca9557_init();
    es8311_init();
    es8311_obj_t *es_obj;
    es_obj = m_new_obj(es8311_obj_t);
    es_obj->base.type = &audio_es8311_type;
    es_obj->callback = mp_const_none;
    

    return MP_OBJ_FROM_PTR(es_obj);

}
//-------------------------------------------------------------------------------------------------------------------------------------------
static mp_obj_t audio_es8311_deinit()
{
    if(audiodev.state != AUDIO_STATE_IDLE ||is_rec != false){
        audiodev.state = AUDIO_STATE_IDLE;
        is_rec = false;

        vTaskDelay(50);
    }
    return mp_const_none;
}
void es8311_deinit()
{
    if(audiodev.state != AUDIO_STATE_IDLE || is_rec != false){
        audiodev.state = AUDIO_STATE_IDLE;
        is_rec = false;

        vTaskDelay(50);
    }
}
static MP_DEFINE_CONST_FUN_OBJ_KW(audio_es8311_deinit_obj, 0, audio_es8311_deinit);

//-----------------------------------------------------------------------------------------------
static const mp_rom_map_elem_t audio_es8311_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR__name__), MP_ROM_QSTR(MP_QSTR_es8311) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&audio_es8311_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_load), MP_ROM_PTR(&audio_es8311_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_play), MP_ROM_PTR(&audio_es8311_play_obj) },
    { MP_ROM_QSTR(MP_QSTR_continue_play), MP_ROM_PTR(&audio_es8311_continue_play_obj) },
    { MP_ROM_QSTR(MP_QSTR_pause), MP_ROM_PTR(&audio_es8311_pause_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop), MP_ROM_PTR(&audio_es8311_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_volume), MP_ROM_PTR(&audio_es8311_volume_obj) },

    { MP_ROM_QSTR(MP_QSTR_record), MP_ROM_PTR(&audio_es8311_record_obj) },
    { MP_ROM_QSTR(MP_QSTR_record_stop), MP_ROM_PTR(&audio_es8311_record_stop_obj) },
    
};

static MP_DEFINE_CONST_DICT(audio_es8311_locals_dict, audio_es8311_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    audio_es8311_type,
    MP_QSTR_ES8311,
    MP_TYPE_FLAG_NONE,
    print, audio_es8311_print,
    make_new, audio_es8311_make_new,
    locals_dict, &audio_es8311_locals_dict
);





#endif