idf_component_register(SRCS "main.cpp"
                            "radioController.cpp"
                            "debug.cpp"
                            "mpu6050.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES
                       arduino
                       bmp280
                       spi_flash
                       esp_partition
                       "driver"
                       )

#SET(CMAKE_CXX_STANDARD 17)
#target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)

