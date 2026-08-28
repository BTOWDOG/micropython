set(IDF_TARGET esp32s3)

set(MICROPY_PORT_PICLIB y) 
set(MICROPY_PORT_CAMLIB y) #CAM
set(MICROPY_PORT_USB_CAM y) #UVCCAM
set(MICROPY_PORT_WEB_STREAM y) #WEB stream

set(SDKCONFIG_DEFAULTS
    boards/sdkconfig.base
	boards/sdkconfig.ble
    boards/PYWIFI_S3P/sdkconfig.board

)

