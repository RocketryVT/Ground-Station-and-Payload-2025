include(../include/FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040/FREERTOS_KERNEL_import.cmake)

set(TARGET_NAME hello_freertos1)
add_executable(${TARGET_NAME}
        hello_freertos.c
)
target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/..
)
target_include_directories(${TARGET_NAME} PUBLIC ../include)

target_link_libraries(${TARGET_NAME} PRIVATE
        pico_async_context_freertos
        FreeRTOS-Kernel-Heap4
        pico_stdlib
)
if(PICO_CYW43_SUPPORTED)
    # For led support on pico_w
    target_link_libraries(${TARGET_NAME} PRIVATE
            pico_cyw43_arch_none
    )
endif()
target_compile_definitions(${TARGET_NAME} PRIVATE
        configNUMBER_OF_CORES=2
)
pico_add_extra_outputs(${TARGET_NAME})