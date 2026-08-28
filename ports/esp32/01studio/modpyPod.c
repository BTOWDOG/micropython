#include "py/obj.h"
#if MICROPY_ENABLE_PCA9557

#include "pca9557.h"

static const mp_rom_map_elem_t pyPod_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR__NAME__), MP_ROM_QSTR(MP_QSTR_pyPod) },

    { MP_ROM_QSTR(MP_QSTR_PCA9557), MP_ROM_PTR(&pyPod_pca9557_type) },
};
static MP_DEFINE_CONST_DICT(pyPod_module_globals, pyPod_module_globals_table);

const mp_obj_module_t pyPod_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&pyPod_module_globals,
};

MP_REGISTER_EXTENSIBLE_MODULE(MP_QSTR_pyPod, pyPod_module);

#endif