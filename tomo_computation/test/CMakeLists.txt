message(STATUS "Building test executables...")

# Test data object
add_executable(test_data_object test_data_object.cpp)
target_link_libraries(test_data_object tomo_computation)
install(TARGETS test_data_object DESTINATION lib/${PROJECT_NAME})
#tomo_test_compte ros_package
add_subdirectory(tomo_test_compute)
# Test conversion
# add_executable(test_conversion test_conversion.cpp)
# target_link_libraries(test_conversion tomo_computation)
# install(TARGETS test_conversion DESTINATION lib/${PROJECT_NAME})

