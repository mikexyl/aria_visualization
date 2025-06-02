cmake_minimum_required(VERSION 3.22) # for cmake presets support
project(aria_viz VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-narrowing")
add_compile_options(-Wno-unused-parameter -Wno-unused-function)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Download the rerun_sdk
find_package(aria_common REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SFML COMPONENTS Graphics REQUIRED)
find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)

include(FetchContent)

find_package(rerun_sdk QUIET)

if(NOT rerun_sdk_FOUND)
  message(STATUS "rerun_sdk not found, downloading...")
  FetchContent_Declare(rerun_sdk URL
    https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
  FetchContent_MakeAvailable(rerun_sdk)
  # Export rerun_sdk target if it exists
  if(TARGET rerun_sdk)
    export(TARGETS rerun_sdk
      FILE "${CMAKE_CURRENT_BINARY_DIR}/rerun_sdkTargets.cmake"
    )
  endif()
else()
  message(STATUS "rerun_sdk found: ${rerun_sdk_INCLUDE_DIRS}")
  message(STATUS "rerun_sdk version: ${rerun_sdk_VERSION}")

endif()

if(rerun_sdk_VERSION VERSION_GREATER 0.23)
  message(STATUS "rerun_sdk version is newer than 0.23")
else()
  message(STATUS "rerun_sdk version is 0.22 or older")
  add_definitions(-DRERUN_SDK_OLD_VERSION)
endif()


# set TGUI_BACKEND to SFML
set(TGUI_BACKEND "SFML_GRAPHICS")
find_package(TGUI REQUIRED)

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
  tgui
  ${TBB_LIBRARIES}
  ${GVC_LIBRARIES})

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

add_subdirectory(tests)

# Export rerun_sdk
# export(EXPORT rerun_sdkTargets FILE _deps/rerun_sdk-build/rerun_sdkTargets.cmake)

# export tgui
if(NOT TGUI_FOUND)
export(TARGETS tgui
  FILE _deps/tgui-build/tguiTargets.cmake
)
endif()

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
