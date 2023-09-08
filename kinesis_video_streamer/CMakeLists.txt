cmake_minimum_required(VERSION 3.5)
project(kinesis_video_streamer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(kinesis_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aws_ros2_common REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(aws_common REQUIRED)
find_package(image_transport REQUIRED)
find_package(kinesis_video_msgs REQUIRED)

set(KINESIS_VIDE_STREAMER_LIBRARY_TARGET ${PROJECT_NAME}_lib)

###########
## Build ##
###########
add_library(${KINESIS_VIDE_STREAMER_LIBRARY_TARGET}
        src/ros_stream_subscription_installer.cpp
        src/subscriber_callbacks.cpp
        src/streamer.cpp
        )
ament_target_dependencies(${KINESIS_VIDE_STREAMER_LIBRARY_TARGET}
        ament_cmake
        rclcpp
        image_transport
        aws_common
        aws_ros2_common
        kinesis_manager
        kinesis_video_msgs
        )

## Specify additional locations of header files
include_directories(
        include
        )

## Declare a C++ executable
add_executable(${PROJECT_NAME}
        src/main.cpp
        )
target_link_libraries(${PROJECT_NAME}
        ${KINESIS_VIDE_STREAMER_LIBRARY_TARGET}
        )

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} ${KINESIS_VIDE_STREAMER_LIBRARY_TARGET}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )
install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h"
        )
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
        )
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch
        )

ament_export_include_directories(include)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rmw_implementation)
ament_export_dependencies(image_transport)
ament_export_dependencies(aws_common)
ament_export_dependencies(aws_ros2_common)
ament_export_dependencies(kinesis_manager)
ament_export_dependencies(kinesis_video_msgs)

ament_package()

#############
## Tests ##
#############
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  
  ament_add_gtest(test_kinesis_video_streamer
    test/kinesis_video_streamer_test.cpp)
  target_include_directories(test_kinesis_video_streamer
    PRIVATE include ${KPL_INCLUDES_PATH}
    ${kinesis_manager_INCLUDE_DIRS})
  target_link_libraries(test_kinesis_video_streamer
          ${KINESIS_VIDE_STREAMER_LIBRARY_TARGET})

  ament_add_gmock(test_streamer_node
    test/streamer_node_test.cpp)
  target_include_directories(test_streamer_node
    PRIVATE include ${KPL_INCLUDES_PATH}
    ${kinesis_manager_INCLUDE_DIRS})
  target_link_libraries(test_streamer_node
          ${KINESIS_VIDE_STREAMER_LIBRARY_TARGET})

  ament_add_gmock(test_subscriber_callbacks
    test/subscriber_callbacks_test.cpp)
  target_include_directories(test_subscriber_callbacks
    PRIVATE include ${KPL_INCLUDES_PATH}
    ${kinesis_manager_INCLUDE_DIRS})
  target_link_libraries(test_subscriber_callbacks
          ${KINESIS_VIDE_STREAMER_LIBRARY_TARGET})
endif()
