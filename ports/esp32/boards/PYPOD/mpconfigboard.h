#define MICROPY_HW_BOARD_NAME               "01Studio pyPod with ESP32S3-N16R8"
#define MICROPY_HW_MCU_NAME                 "ESP32S3"

#define MICROPY_PY_MACHINE_DAC          (0)


#define MICROPY_PY_PICLIB				(1)

#define MICROPY_ENABLE_TFTLCD			(1)
#define MICROPY_STRING_SIZE_24			(1)
#define MICROPY_STRING_SIZE_32			(1)
#define MICROPY_STRING_SIZE_48			(1)

#define MICROPY_HW_LCD15				(1)	

#define MICROPY_ENABLE_PCA9557          (1)
#define MICROPY_ENABLE_SENSOR           (1)

#define LCD_CS_PIN                      BIT(0)
#define DVP_PWDN_PIN                    BIT(1)
#define PA_EN_PIN                       BIT(2)

#define LCD_PIN_DC						(38)
#define LCD_PIN_RST						(-1)
#define LCD_PIN_CS						(-1)
#define LCD_PIN_CLK						(45)
#define LCD_PIN_MISO					(-1)
#define LCD_PIN_MOSI					(39)
#define LCD_BACKLIGHT                   (48)  

#define MICROPY_ENABLE_TOUCH            (1)
#define MICROPY_HW_CST816S              (1)

#define MICROPY_ENABLE_GUI				(1)
#define MICROPY_GUI_BUTTON				(1)
#define GUI_BTN_NUM_MAX					(20)
#define GUI_BTN_STR_LEN					(20)

#define MICROPY_ENABLE_SENSOR           (1)
#define MICROPY_HW_GC0308               (1)
#define MICROPY_ENABLE_STREAM           (1)

#define CAMERA_PIN_PWDN  -1
#define CAMERA_PIN_RESET -1
#define CAMERA_PIN_XCLK   4
#define CAMERA_PIN_SIOD  -1
#define CAMERA_PIN_SIOC   2

#define CAMERA_PIN_D7 3
#define CAMERA_PIN_D6 5
#define CAMERA_PIN_D5 6
#define CAMERA_PIN_D4 15
#define CAMERA_PIN_D3 17
#define CAMERA_PIN_D2 8
#define CAMERA_PIN_D1 18
#define CAMERA_PIN_D0 16
#define CAMERA_PIN_VSYNC 9
#define CAMERA_PIN_HREF 46
#define CAMERA_PIN_PCLK 7

#define XCLK_FREQ_HZ 24000000

#define MICROPY_HW_ESPAI				(1)
#define MICROPY_ENABLE_FACE_DETECTION	(1)
#define MICROPY_ENABLE_CAT_DETECTION	(1)
#define MICROPY_ENABLE_COLOR_DETECTION	(1)
#define MICROPY_ENABLE_CODE_RECOGNITION	(1)
#define MICROPY_ENABLE_MOTION_DETECTION	(1)
#define MICROPY_ENABLE_FACE_RECOGNITION (1)

#define MICROPY_ENABLE_AUDIO           (1)
#define MICROPY_HW_ES8311              (1)

#define BSP_I2S_NUM            0
#define AUDIO_I2S_GPIO_MCLK    12
#define AUDIO_I2S_GPIO_WS      21
#define AUDIO_I2S_GPIO_BCLK    13
#define AUDIO_I2S_GPIO_DIN     47
#define AUDIO_I2S_GPIO_DOUT    14
#define AUDIO_INPUT_SAMPLE_RATE  24000 
#define AUDIO_OUTPUT_SAMPLE_RATE 24000



#define MICROPY_HW_ENABLE_UART_REPL     (1)
#define MICROPY_HW_ENABLE_USBDEV        (1)
#define MICROPY_HW_USB_CDC               1
#define MICROPY_HW_ESP_USB_SERIAL_JTAG   0
