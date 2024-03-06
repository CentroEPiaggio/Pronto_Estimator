# CMake generated Testfile for 
# Source directory: /home/ros/docker_pronto_ws/src/motion_control/whole_body_control/hierarchical_optimization
# Build directory: /home/ros/docker_pronto_ws/build/hierarchical_optimization
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(hierarchical_optimization_test "/usr/bin/python3.10" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/ros/docker_pronto_ws/build/hierarchical_optimization/test_results/hierarchical_optimization/hierarchical_optimization_test.gtest.xml" "--package-name" "hierarchical_optimization" "--output-file" "/home/ros/docker_pronto_ws/build/hierarchical_optimization/ament_cmake_gtest/hierarchical_optimization_test.txt" "--command" "/home/ros/docker_pronto_ws/build/hierarchical_optimization/hierarchical_optimization_test" "--gtest_output=xml:/home/ros/docker_pronto_ws/build/hierarchical_optimization/test_results/hierarchical_optimization/hierarchical_optimization_test.gtest.xml")
set_tests_properties(hierarchical_optimization_test PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/ros/docker_pronto_ws/build/hierarchical_optimization/hierarchical_optimization_test" TIMEOUT "60" WORKING_DIRECTORY "/home/ros/docker_pronto_ws/build/hierarchical_optimization" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/ros/docker_pronto_ws/src/motion_control/whole_body_control/hierarchical_optimization/CMakeLists.txt;78;ament_add_gtest;/home/ros/docker_pronto_ws/src/motion_control/whole_body_control/hierarchical_optimization/CMakeLists.txt;0;")
subdirs("gtest")
