set(IDF_TARGET esp32s3)

set(MICROPY_PORT_DRONE y) #CAM
set(MICROPY_PORT_PICLIB n) #
set(MICROPY_PORT_CAMLIB n) #CAM
set(MICROPY_PORT_WEB_STREAM n) #WEB stream
# boards/sdkconfig.usb
set(SDKCONFIG_DEFAULTS
	boards/sdkconfig.base
	boards/sdkconfig.ble
    boards/PYDRONE_V1.1/sdkconfig.board
	# boards/sdkconfig.cam
)

# if(NOT MICROPY_FROZEN_MANIFEST)
#     set(MICROPY_FROZEN_MANIFEST ${MICROPY_PORT_DIR}/boards/manifest.py)
# endif()
