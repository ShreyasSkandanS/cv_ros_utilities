cmake_minimum_required(VERSION 2.8)
project(cvrosutils)

find_package(catkin REQUIRED COMPONENTS roscpp 
					rospy 
					std_msgs 
					genmsg 
					cv_bridge 
					sensor_msgs 
					image_transport 
					pcl_ros)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

catkin_package(INCLUDE_DIRS include
               LIBRARIES cvrosutils
               CATKIN_DEPENDS message_filters pcl_ros roscpp sensor_msgs cv_bridge
               DEPENDS OpenCV)

option(SHARED_LIB "Request build of shared libraries." ON)
if (SHARED_LIB)
  set(LIB_TYPE SHARED)
else ()
  set(LIB_TYPE STATIC)
endif ()

include_directories(${THIRD_PARTY_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include)
include_directories(${catkin_INCLUDE_DIRS} 
                    ${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include/cvrosutils)
aux_source_directory(${PROJECT_SOURCE_DIR}/src/cvrosutils SOURCE_LIST)
add_library(cvrosutils ${LIB_TYPE} ${SOURCE_LIST})
target_link_libraries(cvrosutils ${catkin_LIBRARIES} ${OpenCV_LIBS})

install(DIRECTORY ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})	
