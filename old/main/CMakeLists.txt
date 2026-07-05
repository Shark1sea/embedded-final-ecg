idf_component_register(
    SRCS  
    "main.c" 
    "panTompkins/panTompkins.c"
    PRIV_REQUIRES 
        spi_flash 
        esp_adc 
        esp_lcd
        fatfs
    INCLUDE_DIRS 
        "."
)

