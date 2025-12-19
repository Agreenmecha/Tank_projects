cmake_minimum_required(VERSION 3.8)
project(master_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  config
  launch
  meshes
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install textures directory if it exists
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/textures")
  install(DIRECTORY
    textures
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_package()
