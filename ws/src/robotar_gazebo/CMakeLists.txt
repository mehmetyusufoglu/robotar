cmake_minimum_required(VERSION 3.22)
project(robotar_gazebo)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(ORCHARD_CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/config/orchard.yaml)
set(GENERATED_WORLD ${CMAKE_CURRENT_BINARY_DIR}/generated/worlds/robotar_orchard.sdf)
set(GENERATED_BRIDGE ${CMAKE_CURRENT_BINARY_DIR}/generated/config/bridge.yaml)

# The orchard world and the ros_gz bridge configuration are generated from
# config/orchard.yaml (single source of truth). Generated files are build
# artifacts and must never be hand-edited.
add_custom_command(
  OUTPUT ${GENERATED_WORLD} ${GENERATED_BRIDGE}
  COMMAND ${Python3_EXECUTABLE}
          ${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_orchard.py
          --config ${ORCHARD_CONFIG}
          --world-out ${GENERATED_WORLD}
          --bridge-out ${GENERATED_BRIDGE}
  DEPENDS
          ${CMAKE_CURRENT_SOURCE_DIR}/tools/generate_orchard.py
          ${ORCHARD_CONFIG}
  COMMENT "Generating orchard SDF world and bridge config from orchard.yaml"
  VERBATIM
)
add_custom_target(robotar_orchard_generation ALL
  DEPENDS ${GENERATED_WORLD} ${GENERATED_BRIDGE}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES config/orchard.yaml config/controllers.yaml
  DESTINATION share/${PROJECT_NAME}/config
)
install(
  FILES ${GENERATED_WORLD}
  DESTINATION share/${PROJECT_NAME}/worlds
)
install(
  FILES ${GENERATED_BRIDGE}
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
