cmake_minimum_required(VERSION 3.8)
project(transformpcd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Set up include directories
include_directories(include)
include_directories(${pcl_ros_INCLUDE_DIRS})
# Add executable
add_executable(transformpcd src/transformpcd.cpp)
# Link against required libraries
ament_target_dependencies(transformpcd rclcpp sensor_msgs tf2_ros pcl_conversions pcl_ros)
# Install executable
install(TARGETS transformpcd 
        DESTINATION lib/${PROJECT_NAME}
        )
# Install launch files
install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
#Install package.xml
ament_package()
