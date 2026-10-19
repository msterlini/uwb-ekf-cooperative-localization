cmake_minimum_required(VERSION 3.8)
project(dist_project)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UwbData.msg"
  "msg/RobotData.msg"
)

# Estimation modules, installed in the Python package of the messages
install(DIRECTORY src/${PROJECT_NAME}/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  PATTERN "__init__.py" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# ROS nodes
install(PROGRAMS
  scripts/uwb_dist_sim.py
  scripts/motion_planner.py
  scripts/kalman_localization.py
  scripts/image_processing.py
  scripts/target_estimator.py
  scripts/live_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch urdf mesh models worlds coordinates
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
