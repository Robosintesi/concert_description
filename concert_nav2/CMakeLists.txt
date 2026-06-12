cmake_minimum_required(VERSION 3.8)
project(concert_nav2)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(aruco_opencv_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(aruco_localization
	src/aruco_localization/aruco_localization.cpp
	src/aruco_localization/main.cpp)
target_include_directories(aruco_localization PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>)
ament_target_dependencies(aruco_localization
	rclcpp angles geometry_msgs tf2 tf2_ros tf2_eigen aruco_opencv_msgs)

target_link_libraries(aruco_localization Eigen3::Eigen yaml-cpp)

install(TARGETS aruco_localization
	DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
	DESTINATION include)

install(
	DIRECTORY launch config maps rviz
	DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	set(ament_cmake_copyright_FOUND TRUE)
	set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
