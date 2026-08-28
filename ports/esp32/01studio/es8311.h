#ifndef MICROPY_INCLUDED_ES8311_H
#define MICROPY_INCLUDED_ES8311_H
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "driver/i2s_std.h"

#include "esp_vfs_fat.h"
#include "py/stream.h"
extern const mp_obj_type_t audio_es8311_type;

typedef enum{
    AUDIO_STATE_IDLE = 0,
    AUDIO_STATE_PAUSE,
    AUDIO_STATE_STOP,
    AUDIO_STATE_PLAY,
}audio_state_t;

typedef struct 
{

    i2s_chan_handle_t tx_handle;
    i2s_chan_handle_t rx_handle;

    const audio_codec_data_if_t* data_if ;
    const audio_codec_ctrl_if_t* ctrl_if ;
    const audio_codec_if_t* codec_if ;
    const audio_codec_gpio_if_t* gpio_if ;
    
    esp_codec_dev_handle_t dev;

    char file_path[50];
    uint8_t len;

    mp_obj_t file;
    const mp_stream_p_t *stream;

    bool input_enabled;
    audio_state_t state;                   

}__audiodev;

typedef struct {
    char riff[4];         // "RIFF"
    uint32_t file_size;  // 文件总长度 - 8
    char wave[4];        // "WAVE"
    char fmt[4];         // "fmt "
    uint32_t fmt_size;  // 16
    uint16_t audio_format; // 1 = PCM
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
    char data[4];       // "data"
    uint32_t data_size; // PCM数据长度
} wav_header_t;




extern __audiodev audiodev;


#endif