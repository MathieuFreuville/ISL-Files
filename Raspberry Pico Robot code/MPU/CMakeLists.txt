add_library( MPU
    MPU.h
    MPU.c
    )

# Add the standard library to the build
target_link_libraries(MPU
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(MPU PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# Add any user requested libraries
target_link_libraries(MPU 
        hardware_i2c
        )