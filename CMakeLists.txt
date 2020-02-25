cmake_minimum_required(VERSION 2.8.3)
project(rt_interface_py)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        rosconsole
        std_msgs
        tf
        nav_msgs
        message_generation
        rospy
        )




## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()



catkin_package(
#        INCLUDE
#        LIBRARIES
#        CATKIN_DEPENDS  tf geometry_msgs rosbag nav_msgs
)

catkin_install_python(PROGRAMS scripts/rt_interface.py  scripts/udp_server.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

include_directories(
        ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(marker_feedback ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(marker_feedback_node src/marker_feedback_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(marker_feedback_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(teb_local_planner_tutorials_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
        scripts/rt_interface.py
        scripts/udp_server.py
        scripts/cmd_vel_publisher.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
message("Catkin package bin destination")
message(${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY  launch scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_teb_local_planner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
