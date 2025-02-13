if (COMPILE_TOOLS)
    add_executable(read_flash
            read_flash.c
            ../src/spi_flash.c
    )

    add_executable(servo_test
            servo_test.cpp
            ../src/pwm.cpp
    )

    add_executable(alt_test
            alt_test.cpp
    )

    add_executable(imu_calib
            imu_calib.cpp
    )

    add_executable(uart_usb_ftdi_cable
        uart_usb_ftdi_cable.cpp
    )

    add_executable(gps_test
        gps_test.cpp
        ./GPS/crc.cpp
        ./GPS/gps_helper.cpp
        ./GPS/nmea.cpp
        ./GPS/rtcm.cpp
        ./GPS/unicore.cpp
    )

    add_executable(hello_world
            hello_world.cpp
    )

    # pull in common dependencies
    target_link_libraries(read_flash pico_stdlib hardware_spi)
    target_include_directories(read_flash PUBLIC ../include)

    target_link_libraries(servo_test pico_stdlib hardware_pwm hardware_i2c)
    target_include_directories(servo_test PUBLIC ../include)

    target_link_libraries(alt_test pico_stdlib hardware_i2c hardware_gpio)

    target_link_libraries(imu_calib pico_stdlib hardware_i2c hardware_gpio)
    target_include_directories(imu_calib PUBLIC ../include)

    target_link_libraries(uart_usb_ftdi_cable pico_stdlib hardware_uart)
    target_include_directories(uart_usb_ftdi_cable PUBLIC ../include)

    target_link_libraries(gps_test pico_stdlib hardware_uart)
    target_include_directories(gps_test PUBLIC ../include)

    target_link_libraries(hello_world pico_stdlib)
    target_include_directories(hello_world PUBLIC ../include)

    pico_enable_stdio_usb(read_flash 1)
    pico_enable_stdio_uart(read_flash 0)

    pico_enable_stdio_usb(servo_test 1)
    pico_enable_stdio_uart(servo_test 0)

    pico_enable_stdio_usb(alt_test 1)
    pico_enable_stdio_uart(alt_test 0)

    pico_enable_stdio_usb(imu_calib 1)
    pico_enable_stdio_uart(imu_calib 0)

    pico_enable_stdio_usb(uart_usb_ftdi_cable 1)
    pico_enable_stdio_uart(uart_usb_ftdi_cable 0)

    pico_enable_stdio_usb(gps_test 1)
    pico_enable_stdio_uart(gps_test 0)

    pico_enable_stdio_usb(hello_world 1)
    pico_enable_stdio_uart(hello_world 0)

    # create map/bin/hex file etc.
    pico_add_extra_outputs(read_flash)
    pico_add_extra_outputs(servo_test)
    pico_add_extra_outputs(alt_test)
    pico_add_extra_outputs(imu_calib)
    pico_add_extra_outputs(uart_usb_ftdi_cable)
    pico_add_extra_outputs(gps_test)
    pico_add_extra_outputs(hello_world)
endif()

