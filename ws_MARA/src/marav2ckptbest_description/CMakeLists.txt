cmake_minimum_required(VERSION 3.8)
project(marav2ckptbest_description)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(marav2ckptbest_hardware SHARED
  src/marav2ckptbest_hardware.cpp
)
target_compile_features(marav2ckptbest_hardware PUBLIC cxx_std_17)
ament_target_dependencies(marav2ckptbest_hardware
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface hardware_plugin.xml)

install(TARGETS marav2ckptbest_hardware
  LIBRARY DESTINATION lib
)

install(FILES hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install package data
install(DIRECTORY urdf meshes launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
