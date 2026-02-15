cmake_minimum_required(VERSION 3.8)
project(shell_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install the launch directory to share/shell_bringup/launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()