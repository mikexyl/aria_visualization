cmake_minimum_required(VERSION 3.16) # for cmake presets support
project(aria_viz VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-narrowing")
add_compile_options(-Wno-unused-parameter -Wno-unused-function)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Download the rerun_sdk
find_package(aria_common REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)
find_package(spdlog REQUIRED)

include(FetchContent)

find_package(rerun_sdk REQUIRED)

# Find Graphviz using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GVC REQUIRED libgvc)

set(INCLUDES
  ${OpenCV_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
  ${GVC_INCLUDE_DIRS})

set(DEPENDENCIES
  rerun_sdk
  ${OpenCV_LIBS}
  gtsam
  aria_common
  ${TBB_LIBRARIES}
  spdlog::spdlog
  )

add_library(${PROJECT_NAME} STATIC
  src/visualizer.cpp
  src/visualizer_rerun.cpp
)

# Ensure position independent code for static lib to allow linking into shared libs
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

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

add_subdirectory(tests)

# Export rerun_sdk
# export(EXPORT rerun_sdkTargets FILE _deps/rerun_sdk-build/rerun_sdkTargets.cmake)

# Export the target for use in other projects
export(TARGETS ${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

# Skip installing headers since they are used in-place
# Install the export file for the target
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
)

install(DIRECTORY include/aria_viz
  DESTINATION include)

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
