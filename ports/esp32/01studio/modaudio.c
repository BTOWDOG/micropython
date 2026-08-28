
#include "py/obj.h"
#if MICROPY_ENABLE_AUDIO

#if MICROPY_HW_ES8311
#include "es8311.h"
#endif

static const mp_rom_map_elem_t audio_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR__name__), MP_ROM_QSTR(MP_QSTR_audio) },
    
    #if MICROPY_HW_ES8311
    { MP_ROM_QSTR(MP_QSTR_ES8311), MP_ROM_PTR(&audio_es8311_type) },
    #endif
};
static MP_DEFINE_CONST_DICT(audio_module_globals, audio_module_globals_table);

const mp_obj_module_t audio_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&audio_module_globals,
};
//------------------------------------------------------------------------------------
MP_REGISTER_EXTENSIBLE_MODULE(MP_QSTR_audio, audio_module);
#endif