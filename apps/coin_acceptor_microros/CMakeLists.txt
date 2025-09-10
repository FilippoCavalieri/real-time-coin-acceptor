# Define the executable for the Pico Micro-ROS example
add_executable(coin_acceptor_microros
    main.cpp
    lib/PhotoresistorSensor.cpp
    lib/ServoActuator.cpp
    lib/StrainGaugeSensor.cpp
    lib/LM393CoupleSensor.cpp
)

# Link against Pico, FreeRTOS, microros, and shared utilities
target_link_libraries(coin_acceptor_microros
    pico_stdlib
    common_utils
    hardware_adc
    hardware_gpio
    hardware_pwm
    hardware_uart
    FreeRTOS-Kernel FreeRTOS-Kernel-Heap4
    microros
)

# Configure Pico IO settings
pico_enable_stdio_usb(coin_acceptor_microros 1)
pico_enable_stdio_uart(coin_acceptor_microros 0)

# Set UF2 output directory
set_target_properties(coin_acceptor_microros PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Generate UF2 and binary outputs
pico_add_extra_outputs(coin_acceptor_microros)