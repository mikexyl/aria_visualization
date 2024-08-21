cmake_minimum_required(VERSION 3.23) # for cmake presets support
project(aria_visualization VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Download the rerun_sdk
include(FetchContent)
FetchContent_Declare(rerun_sdk URL
  https://github.com/rerun-io/rerun/releases/download/0.17.0/rerun_cpp_sdk.zip)
FetchContent_MakeAvailable(rerun_sdk)

find_package(aria_net REQUIRED)
find_package(OpenCV REQUIRED)

set(INCLUDES
  ${aria_net_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(DEPENDENCIES
  ${aria_net_LIBRARIES}
  rerun_sdk
  ${OpenCV_LIBS}
)
add_library(${PROJECT_NAME} STATIC
  src/frame_visualizer_rerun.cpp
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
  ${DEPENDENCIES}
)

# Export rerun_sdk
export(EXPORT rerun_sdkTargets FILE rerun_sdkTargets.cmake)

# Export the target for use in other projects
export(TARGETS ${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets # Export the target for use in other projects
  LIBRARY DESTINATION lib # Where to install the shared library (.so/.dll/.dylib)
  ARCHIVE DESTINATION lib # Where to install static libraries (if any)
  RUNTIME DESTINATION bin # Where to install executables (if any)
)

# Skip installing headers since they are used in-place
# Install the export file for the target
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

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
