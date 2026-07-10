cmake_minimum_required(VERSION 3.16)
project(car_host_tests C)

add_executable(test_control
    test_control.c
    ../../src/control/PID_core.c
    ../../src/control/control_math.c)
target_include_directories(test_control PRIVATE ../../src/control)
target_compile_features(test_control PRIVATE c_std_99)
if(MSVC)
    target_compile_options(test_control PRIVATE /W4)
else()
    target_compile_options(test_control PRIVATE -Wall -Wextra -Werror)
    target_link_libraries(test_control PRIVATE m)
endif()

enable_testing()
add_test(NAME control COMMAND test_control)
