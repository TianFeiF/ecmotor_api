cmake_minimum_required(VERSION 3.15)
project(ecmotor_api CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(test
  test.cpp
  src/motor_api.cpp
  src/motor_adapter.cpp
  src/vendor_adapters.cpp
)

target_include_directories(test PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  /usr/local/etherlab/include
)

find_library(ECRT_LIB ethercat HINTS /usr/local/etherlab/lib /usr/local/lib /usr/lib /usr/lib64)
if (NOT ECRT_LIB)
    message(FATAL_ERROR "ethercat library not found")
endif()

target_link_libraries(test ${ECRT_LIB})

set_target_properties(test PROPERTIES
  BUILD_RPATH "/usr/local/etherlab/lib;/usr/local/lib"
)

# 添加ENI测试程序
add_executable(test_eni
  test_eni.cpp
  src/motor_api.cpp
  src/motor_adapter.cpp
  src/vendor_adapters.cpp
)

target_include_directories(test_eni PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  /usr/local/etherlab/include
)

target_link_libraries(test_eni ${ECRT_LIB})

set_target_properties(test_eni PROPERTIES
  BUILD_RPATH "/usr/local/etherlab/lib;/usr/local/lib"
)

# 添加路径播放测试程序
add_executable(test_path_playback
  test_path_playback.cpp
  src/motor_api.cpp
  src/motor_adapter.cpp
  src/vendor_adapters.cpp
)

target_include_directories(test_path_playback PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  /usr/local/etherlab/include
)

target_link_libraries(test_path_playback ${ECRT_LIB})

set_target_properties(test_path_playback PROPERTIES
  BUILD_RPATH "/usr/local/etherlab/lib;/usr/local/lib"
)

# 添加调试测试程序
add_executable(test_debug
  test_debug.cpp
  src/motor_api.cpp
  src/motor_adapter.cpp
  src/vendor_adapters.cpp
)

target_include_directories(test_debug PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  /usr/local/etherlab/include
)

target_link_libraries(test_debug ${ECRT_LIB})

set_target_properties(test_debug PROPERTIES
  BUILD_RPATH "/usr/local/etherlab/lib;/usr/local/lib"
)

add_custom_target(copy_compile_commands ALL
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_BINARY_DIR}/compile_commands.json
          ${CMAKE_SOURCE_DIR}/compile_commands.json
  DEPENDS ${CMAKE_BINARY_DIR}/compile_commands.json
  COMMENT "Copying compile_commands.json to source directory"
)