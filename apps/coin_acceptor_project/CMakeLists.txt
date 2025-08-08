# Define the executable for the Pico Micro-ROS example
add_executable(coin_acceptor_project
    main.cpp
    Fotoresistore.cpp
    Servo.cpp
    Estensimetro.cpp
)

# Link against Pico, FreeRTOS, microros, and shared utilities
target_link_libraries(coin_acceptor_project
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_pwm
    FreeRTOS-Kernel FreeRTOS-Kernel-Heap4
    microros
)

# Configure Pico IO settings
pico_enable_stdio_usb(coin_acceptor_project 1)
pico_enable_stdio_uart(coin_acceptor_project 0)

# Set UF2 output directory
set_target_properties(coin_acceptor_project PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Generate UF2 and binary outputs
pico_add_extra_outputs(coin_acceptor_project)
