idf_component_register(SRCS         
                         "src/esp_crsf.cpp" 
                         "src/i2c.cpp" 
                         "src/main.cpp"
                         "src/radioController.cpp"
                         "src/debug.cpp"
                         "src/mpu6050.cpp"
                         "src/webClient.cpp"
                         "src/drone.cpp"
                         "src/development.cpp"
                         "src/dshot600.cpp"
                         "src/display.cpp"
                         "icons/wifi_icon.cpp"
                         "icons/flight_icon.cpp"
                         "icons/sensor_connected_icon.cpp"
                       INCLUDE_DIRS 
                         "."
                         "includes"
                         "icons"
                         "src"
                       REQUIRES
                         arduino
                         bmp280
                         spi_flash
                         esp_partition
                         esp_wifi
                         esp_rom
                         nvs_flash
                         "driver"
                        "nvs_flash"
                         
                       )

#SET(CMAKE_CXX_STANDARD 17)
#target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)

