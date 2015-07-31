cmake_minimum_required(VERSION 2.8.3)
project(fetch_tools)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES setup.bash
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/fetch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_add_env_hooks(30.fetch_tools SHELLS bash DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
