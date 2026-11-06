cmake_minimum_required(VERSION 3.8)
project(pouk_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_urdf_matches_kinematics test/test_urdf_matches_kinematics.py)
endif()

ament_package()
