cmake_minimum_required(VERSION 3.8)
project(balance_car_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(PROGRAMS
  src/imu_debug_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME imu_debug_node
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

install(PROGRAMS
  src/balance_controller_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME balance_controller_node
)

install(FILES
  src/balance_types.py
  src/attitude_estimator.py
  src/balance_pd.py
  src/velocity_loop.py
  src/motion_command.py
  src/yaw_loop.py
  src/safety_limiter.py
  src/wheel_mixer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
