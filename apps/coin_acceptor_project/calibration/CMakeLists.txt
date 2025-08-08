# Define the executable for the Pico Micro-ROS example
add_executable(calibration
    main.cpp
    Fotoresistore.cpp
    Servo.cpp
    Estensimetro.cpp
    DimensionSensor.cpp
)

# Link against Pico, FreeRTOS, microros, and shared utilities
target_link_libraries(calibration
    pico_stdlib
    hardware_adc
    hardware_gpio
    hardware_pwm
    FreeRTOS-Kernel FreeRTOS-Kernel-Heap4
    microros
)

# Configure Pico IO settings
pico_enable_stdio_usb(calibration 1)
pico_enable_stdio_uart(calibration 0)

# Set UF2 output directory
set_target_properties(calibration PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Generate UF2 and binary outputs
pico_add_extra_outputs(calibration)
