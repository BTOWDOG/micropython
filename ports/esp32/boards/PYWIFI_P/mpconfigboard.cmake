set(MICROPY_PORT_PICLIB y) #

set(SDKCONFIG_DEFAULTS
	boards/PYWIFI_P/sdkconfig.board
    boards/sdkconfig.ble
    boards/sdkconfig.spiram_quad
	boards/sdkconfig.240mhz
)

# if(NOT MICROPY_FROZEN_MANIFEST)
#     set(MICROPY_FROZEN_MANIFEST ${MICROPY_PORT_DIR}/boards/manifest.py)
# endif()
