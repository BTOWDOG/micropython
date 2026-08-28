#include "py/obj.h"
#include "py/runtime.h"
#include "mod_balance.h"


#if MICROPY_ENABLE_ESP_PYBALANCE 

static const mp_rom_map_elem_t espbalance_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR__name__), MP_ROM_QSTR(MP_QSTR_espbalance) },
    { MP_ROM_QSTR(MP_QSTR_BALANCE), MP_ROM_PTR(&balance_balance_type) },
};

static MP_DEFINE_CONST_DICT(espbalance_module_globals, espbalance_module_globals_table);

const mp_obj_module_t espbalance_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&espbalance_module_globals,
};

MP_REGISTER_EXTENSIBLE_MODULE(MP_QSTR_pyBalance, espbalance_module);

#endif