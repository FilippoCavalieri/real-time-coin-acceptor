# Define the executable for the Pico Micro-ROS example
add_executable(my_coin_acceptor
    main.cpp
    lib/PhotoresistorSensor.cpp
    lib/ServoActuator.cpp
    lib/StrainGaugeSensor.cpp
    lib/LM393CoupleSensor.cpp
)

# Link against Pico, FreeRTOS, microros, and shared utilities
target_link_libraries(my_coin_acceptor
    pico_stdlib
    common_utils
    hardware_adc
    hardware_gpio
    hardware_pwm
    hardware_uart
    microros
    FreeRTOS-Kernel FreeRTOS-Kernel-Heap4
)

# Configure Pico IO settings
pico_enable_stdio_usb(my_coin_acceptor 1)
pico_enable_stdio_uart(my_coin_acceptor 0)

# Set UF2 output directory
set_target_properties(my_coin_acceptor PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Generate UF2 and binary outputs
pico_add_extra_outputs(my_coin_acceptor)