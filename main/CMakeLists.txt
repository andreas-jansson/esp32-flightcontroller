idf_component_register(SRCS         
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
                         "src/ringbuffer.cpp"
                         "src/pidConfig.cpp"

                         "icons/wifi_icon.cpp"
                         "icons/flight_icon.cpp"
                         "icons/sensor_connected_icon.cpp"

                         "stub_src/radiocontroller_stub.cpp"
                         "stub_src/mpu6050_stub.cpp"

                       INCLUDE_DIRS 
                         "."
                         "includes"
                         "icons"
                         "src"
                         "stub_src"
                         "stub_includes"
                       REQUIRES
                         arduino
                         bmp280
                         spi_flash
                         esp_partition
                         esp_wifi
                         esp_rom
                         nvs_flash
                         TFT_eSPI
                         "driver"
                         "nvs_flash" 
                       )

#SET(CMAKE_CXX_STANDARD 17)
#target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)

