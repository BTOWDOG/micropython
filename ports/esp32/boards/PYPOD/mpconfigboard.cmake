set(IDF_TARGET esp32s3)

set(MICROPY_PORT_PICLIB y) #
set(MICROPY_PORT_CAMLIB y) #CAM
set(MICROPY_PORT_WEB_STREAM y) #WEB stream
set(MICROPY_PORT_EN_ESPAI y) 
set(MICROPY_PORT_AUDIO_CODEC y)

set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
	boards/sdkconfig.ble
    boards/PYPOD/sdkconfig.board
)

# if(NOT MICROPY_FROZEN_MANIFEST)
#     set(MICROPY_FROZEN_MANIFEST ${MICROPY_PORT_DIR}/boards/manifest.py)
# endif()
