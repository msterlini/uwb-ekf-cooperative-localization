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

# Pure-Python estimation modules in src/dist_project, installed in the same Python package generated for the
# messages (dist_project.msg), so that both dist_project.ekf and dist_project.msg can be imported
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
