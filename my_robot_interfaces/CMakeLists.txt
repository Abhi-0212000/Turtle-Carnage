cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# ==========================> Add these below line for configuration purposes.

find_package(rosidl_default_generators REQUIRED) # i added this line

# ============================> From now on everytime you add a .msg file, you have to add it in below generate interfaces code.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TargetTurtleParams.srv"
  "srv/KillServerParams.srv"
)

ament_export_dependencies(rosidl_default_runtime)


ament_package()
