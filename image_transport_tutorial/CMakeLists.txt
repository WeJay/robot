cmake_minimum_required(VERSION 2.8)
project(image_transport_tutorial)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs cv_bridge image_transport message_generation sensor_msgs tf actionlib
  move_base_msgs)

# add the resized image message
add_message_files(DIRECTORY msg
   FILES
   ResizedImage.msg 
   debug.msg
)
generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs nav_msgs cv_bridge image_transport message_runtime sensor_msgs)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add the cross_cell completecoverage #20180510
add_executable(coverage_test_crosscell src/coverage_test_crosscell.cpp)
add_dependencies(coverage_test_crosscell ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(coverage_test_crosscell ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the subscriber test example # 20180328 
add_executable(coverage_test src/coverage_test.cpp)
add_dependencies(coverage_test ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(coverage_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# add the subscriber test example # 20180328 
add_executable(coverage_demo src/coverage_demo.cpp)
add_dependencies(coverage_demo ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(coverage_demo ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# add the subscriber test example # 20180416
add_executable(coverage_virtualwall src/coverage_virtualwall.cpp)
add_dependencies(coverage_virtualwall ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(coverage_virtualwall ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


# Mark executables and/or libraries for installation
install(TARGETS my_publisher my_subscriber resized_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES resized_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
