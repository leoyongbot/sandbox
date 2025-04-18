cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tomo_computation)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)  # Or use 17, 20, etc.
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#options
option(BUILD_TESTING "Enable building of tests" OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
set(SRC_FILES
	src/data_object.cpp
    src/computation.cpp
)

set(DEPENDS
	ament_cmake
	tf2_geometry_msgs
  	tf2_ros
 	tf2
	Eigen3
	geometry_msgs
	)

include_directories(${PROJECT_NAME}
	PUBLIC
	include
	${tf2_INCLUDE_DIRS}
	${geometry_msgs_INCLUDE_DIRS})

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<INSTALL_INTERFACE:include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_export_include_directories(include)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_target_dependencies(
  ${PROJECT_NAME}
  ${DEPENDS}
)
ament_export_dependencies(${DEPENDS})
#Install libraries and headers
install(DIRECTORY include/
		DESTINATION include)
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
  	ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    INCLUDES DESTINATION include
    )

if(BUILD_TESTING)
	message(STATUS "Building tests...")
	add_subdirectory(test)
endif()
# Package the library
ament_package()