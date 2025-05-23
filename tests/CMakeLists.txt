add_executable(test_rerun rerun.cpp)
target_include_directories(test_rerun PRIVATE ${INCLUDES})
target_link_libraries(test_rerun ${PROJECT_NAME})

# copy the tinyGrid3D.g2o file to the build directory
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tinyGrid3D.g2o ${CMAKE_CURRENT_BINARY_DIR}/tinyGrid3D.g2o COPYONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/smallGrid3D.g2o ${CMAKE_CURRENT_BINARY_DIR}/smallGrid3D.g2o COPYONLY)

