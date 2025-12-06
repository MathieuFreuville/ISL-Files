add_library( Motors
    Motors.h
    Motors.c
    )

# Add the standard library to the build
target_link_libraries(Motors
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Motors PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# Add any user requested libraries
target_link_libraries(Motors 
        hardware_pwm
        )