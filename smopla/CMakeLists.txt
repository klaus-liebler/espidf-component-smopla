idf_component_register(
    SRCS "devicemanager.cc" "http_handlers.cc"
    INCLUDE_DIRS "include"
    REQUIRES "main" "aht_sensor" "common" "driver" "errorcodes" "ds18b20" "esp_http_server" "esp_https_ota" "espidf-ads1115" "espidf-hdc1080" "espidf-rotenc" "espidf-bh1750" "espidf-bme280" "espidf-ccs811" "fft"  "i2c" "i2c_sensor"  "mp3" "rgbled" "spiffs" "pidcontroller" "usb-pd" "wifimanager" 
    EMBED_FILES ../../labathome/labathome_webui/dist_compressed/index.html.gz
    )
