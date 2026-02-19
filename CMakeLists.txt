
    cmake_minimum_required(VERSION 3.8)
    project(sensor_data_evaluation)

    if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
      add_compile_options(-Wall -Wextra -Wpedantic)
    endif()

    # find dependencies
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)

    # add the executable
    add_executable(sensor_node_publisher src/sensor_node_publisher.cpp)
    target_link_libraries(sensor_node_publisher PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

    add_executable(sensor_node_subscriber src/sensor_node_subscriber.cpp)
    target_link_libraries(sensor_node_subscriber PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

    install(TARGETS
      sensor_node_publisher
      sensor_node_subscriber
      DESTINATION lib/${PROJECT_NAME})


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

    ament_package()
            