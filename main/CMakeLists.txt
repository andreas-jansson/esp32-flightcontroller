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
                       INCLUDE_DIRS 
                         "."
                         "includes"
                         "src"
                       REQUIRES
                         arduino
                         bmp280
                         spi_flash
                         esp_partition
                         esp_wifi
                         nvs_flash
                         "driver"
                       )

#SET(CMAKE_CXX_STANDARD 17)
#target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)

