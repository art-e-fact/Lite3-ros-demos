cmake_minimum_required(VERSION 3.10)

project(lite3_sdk_deploy)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
# 警告忽略和编译选项
add_definitions(-w)
add_compile_options(-fPIC)

# 启用 backward-cpp 用于段错误排查
# 定义 USE_SIMULATION 宏以启用 main.cpp 中的 backward 代码
# 定义 BACKWARD_HAS_DW=1 以使用 libdw 进行符号解析

# add_definitions(-DUSE_SIMULATION -DBACKWARD_HAS_DW=1)

# 添加调试信息以便 backward 显示行号

# add_compile_options(-g -rdynamic)

# 配置选项
set(BUILD_PLATFORM "x86" CACHE STRING "Target platform (x86 or arm)")

if(BUILD_PLATFORM STREQUAL "arm")
  set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
  set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++") 
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(drdds REQUIRED)
find_package(geometry_msgs REQUIRED)

if(NOT BUILD_PLATFORM STREQUAL "arm")
  find_library(LIBEVDEV_LIBRARY evdev
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu
    NO_DEFAULT_PATH
  )
  find_path(LIBEVDEV_INCLUDE_DIR libevdev-1.0/libevdev/libevdev.h
    PATHS /usr/include
    NO_DEFAULT_PATH
  )
  if(NOT LIBEVDEV_LIBRARY OR NOT LIBEVDEV_INCLUDE_DIR)
    message(FATAL_ERROR "libevdev not found. Install with: sudo apt install libevdev-dev")
  endif()
  include_directories(${LIBEVDEV_INCLUDE_DIR})
endif()

set(THIRD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

include_directories(
  include/types
  include/utils
  interface/robot
  interface/user_command
  state_machine
  run_policy
  ${THIRD_PARTY}/eigen/
  ${THIRD_PARTY}/gamepad/include
  ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/include
)

# 子目录
add_subdirectory(interface)

# 运行时路径设置
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--enable-new-dtags")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


set(CMAKE_INSTALL_RPATH 
  ${THIRD_PARTY}/onnxruntime/${BUILD_PLATFORM}/lib/
  ../lib/onnxruntime/
)

file(GLOB_RECURSE STATE_MACHINE_SRC "state_machine/*.c*")

add_executable(rl_deploy main.cpp ${STATE_MACHINE_SRC})

add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${THIRD_PARTY}/onnxruntime//${BUILD_PLATFORM}/lib/libonnxruntime.so")

ament_target_dependencies(rl_deploy
  rclcpp
  drdds
  geometry_msgs
)
if(BUILD_PLATFORM STREQUAL "arm")
  target_link_libraries(rl_deploy
    interface
    onnxruntime
    -lpthread
    -lm
    -lrt
    -ldl
    -lstdc++
    # -ldw  启用backward调试功能需要libdw库
  )
else()
  target_link_libraries(rl_deploy
    interface
    onnxruntime
    -lpthread
    -lm
    -lrt
    -ldl
    -lstdc++
    ${LIBEVDEV_LIBRARY}
    # -ldw  启用backward调试功能需要libdw库
  )
endif()


install(TARGETS rl_deploy
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python simulation scripts (runnable via: ros2 run lite3_sdk_deploy mujoco_simulation_ros2.py)
install(PROGRAMS
  interface/robot/simulation/mujoco_simulation_ros2.py
  interface/robot/simulation/auto_waypoint_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  interface/robot/simulation/procedural_scene_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  interface/robot/simulation/sensors
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config
  interface/robot/launch
  Lite3_description
  DESTINATION share/${PROJECT_NAME}
)

ament_package()