cmake_minimum_required(VERSION 3.16)
project(principia_robotica_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(principia_cbf STATIC
    src/cbf_qp_solver.cpp
)
target_include_directories(principia_cbf PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(principia_cbf PRIVATE -O3 -Wall -Wextra -Werror -Wpedantic)

enable_testing()
add_executable(test_cbf_qp tests/test_cbf_qp.cpp)
target_link_libraries(test_cbf_qp PRIVATE principia_cbf)
target_compile_options(test_cbf_qp PRIVATE -O3 -Wall -Wextra -Werror -Wpedantic)
add_test(NAME principia_cpp_cbf_tests COMMAND test_cbf_qp)
