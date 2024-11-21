add_executable(test_rerun rerun.cpp)
target_include_directories(test_rerun PRIVATE ${INCLUDES})
target_link_libraries(test_rerun ${PROJECT_NAME})

add_executable(sfml_to_rerun sfml_rerun.cpp)
target_include_directories(sfml_to_rerun PRIVATE ${INCLUDES})
target_link_libraries(sfml_to_rerun ${PROJECT_NAME})

add_executable(sfml_animation sfml_animation_visualizer.cpp)
target_include_directories(sfml_animation PRIVATE ${INCLUDES})
target_link_libraries(sfml_animation ${PROJECT_NAME})

add_executable(sfml_factor_graph sfml_factor_graph.cpp)
target_include_directories(sfml_factor_graph PRIVATE ${INCLUDES})
target_link_libraries(sfml_factor_graph ${PROJECT_NAME})