cmake_minimum_required(VERSION 2.8.3)
project(lslidar_c32_decoder)

add_compile_options(-std=c++14)
set( CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
        angles
        nodelet
        pcl_ros
        roscpp
        sensor_msgs
        tf
        lslidar_c32_driver
        lslidar_c32_msgs
        dynamic_reconfigure
        pcl_conversions)

find_package(Boost COMPONENTS signals)
find_package(PkgConfig REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES lslidar_c32_decoder
        CATKIN_DEPENDS
        angles
        nodelet
        pcl_ros
        roscpp
        sensor_msgs
        tf
        lslidar_c32_driver
        lslidar_c32_msgs
        dynamic_reconfigure
        pcl_conversions
        DEPENDS
        Boost
)


include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
)

add_library(lslidar_data_c32 src/rawdata.cc)
target_link_libraries(lslidar_data_c32
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES})

add_library(lslidar_point_c32 src/convert.cc)
target_link_libraries(lslidar_point_c32
        lslidar_data_c32
        ${catkin_LIBRARIES})


add_library(lslidar_c32_decoder_nodelet src/cloud_nodelet.cc)
target_link_libraries(lslidar_c32_decoder_nodelet lslidar_point_c32
        ${catkin_LIBRARIES})

add_executable(lslidar_c32_decoder_node src/cloud_node.cc)
target_link_libraries(lslidar_c32_decoder_node
        lslidar_point_c32
        ${catkin_LIBRARIES}
        ${libpcap_LIBRARIES})
add_dependencies(lslidar_c32_decoder_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        )


if(catkin_EXPORTED_TARGETS)
    add_dependencies(lslidar_data_c32 ${catkin_EXPORTED_TARGETS})
endif()




install(TARGETS lslidar_data_c32 lslidar_point_c32 lslidar_c32_decoder_node lslidar_c32_decoder_nodelet
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
nodelets.xml
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
