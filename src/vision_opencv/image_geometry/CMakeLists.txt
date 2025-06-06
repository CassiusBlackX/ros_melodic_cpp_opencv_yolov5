cmake_minimum_required(VERSION 3.0.2)
project(image_geometry)

find_package(catkin REQUIRED sensor_msgs)
find_package(OpenCV REQUIRED)
include_directories(/usr/local/opencv-4.6.0/include/opencv4)

catkin_package(CATKIN_DEPENDS sensor_msgs
               DEPENDS OpenCV
               INCLUDE_DIRS include
               LIBRARIES ${PROJECT_NAME}
)

catkin_python_setup()

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add a library
add_library(${PROJECT_NAME} src/pinhole_camera_model.cpp src/stereo_camera_model.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/
)

# install library
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# add tests
if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
