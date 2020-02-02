find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${ObjectTracking_SOURCE_DIR}/MainProject/inc )
include_directories( ${ObjectTracking_SOURCE_DIR}/LibProject/inc )
link_directories( ${ObjectTracking_SOURCE_DIR}/LibProject/src )

add_executable( runRobot main.cpp )
ament_target_dependencies(runRobot OpenCV rclcpp std_msgs geometry_msgs)
target_include_directories(runRobot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
  
  install(TARGETS runRobot
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})


target_link_libraries ( runRobot LibProject )
target_link_libraries( runRobot ${OpenCV_LIBS} )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()