cmake_minimum_required(VERSION 3.23) # for cmake presets support
project(aria_viz VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-narrowing")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Download the rerun_sdk
find_package(aria_common REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rerun_sdk REQUIRED)

get_target_property(aria_common_LOCATION aria_common LOCATION)
message(STATUS "aria_common_LOCATION: ${aria_common_LOCATION}")

set(INCLUDES
  ${OpenCV_INCLUDE_DIRS}
)

set(DEPENDENCIES
  rerun_sdk
  ${OpenCV_LIBS}
  gtsam
  aria_common
)
add_library(${PROJECT_NAME} STATIC
  src/visualizer.cpp
  src/visualizer_rerun.cpp
)

# Specify include directories
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> # In-place install for headers
  ${INCLUDES}
)

# Specify libraries to link with
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${DEPENDENCIES}
)

add_executable(test_rerun tests/rerun.cpp)
target_include_directories(test_rerun PRIVATE ${INCLUDES})
target_link_libraries(test_rerun ${PROJECT_NAME})

# Export rerun_sdk
# export(EXPORT rerun_sdkTargets FILE _deps/rerun_sdk-build/rerun_sdkTargets.cmake)

# Export the target for use in other projects
export(TARGETS ${PROJECT_NAME} 
  FILE ${PROJECT_NAME}Targets.cmake
)

# install(TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}Targets
#   RUNTIME DESTINATION bin
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   INCLUDES DESTINATION include
# )

# # Skip installing headers since they are used in-place
# # Install the export file for the target
# install(EXPORT ${PROJECT_NAME}Targets
#   FILE ${PROJECT_NAME}Targets.cmake
#   DESTINATION lib/cmake/${PROJECT_NAME}
# )

# Configure and install the package configuration file
include(CMakePackageConfigHelpers)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  COMPATIBILITY AnyNewerVersion
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME}
)
