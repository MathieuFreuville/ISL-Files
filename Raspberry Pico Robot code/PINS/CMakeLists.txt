add_library( Pins
    Pins.h
    Pins.c
    )

# Add the standard library to the build
target_link_libraries(Pins
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Pins PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# Add any user requested libraries
target_link_libraries(Pins 
        hardware_i2c
        hardware_pwm
        )