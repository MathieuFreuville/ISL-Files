add_library( Wifi_Server
    Wifi_Server.h
    Wifi_Server.c
    )

# Add the standard library to the build
target_link_libraries(Wifi_Server
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Wifi_Server PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# Add any user requested libraries
target_link_libraries(Wifi_Server 
        pico_cyw43_arch_none
        pico_stdlib
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_http
        )