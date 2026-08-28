set(IDF_TARGET esp32s3)

set(MICROPY_PORT_PICLIB y) #
set(MICROPY_PORT_BALANCE y)

set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
    boards/sdkconfig.ble
    boards/PYBALANCE/sdkconfig.board
)