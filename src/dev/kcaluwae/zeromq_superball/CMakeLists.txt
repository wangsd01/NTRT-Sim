find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs geometry_msgs)

project(kcaluwaet6)


file(GLOB_RECURSE SRC_KCAL ${CMAKE_CURRENT_SOURCE_DIR}/controllers/*.cpp)

add_library( ${PROJECT_NAME} SHARED ${SRC_KCAL} T6Model.cpp)
target_link_libraries(${PROJECT_NAME} tgcreator controllers core)

link_directories(${LIB_DIR})
link_libraries(tgcreator controllers core)

#add_executable(AppSUPERballGPS
#    T6Model.cpp
#    AppSUPERball.cpp
#    controllers/T6PIDController.cpp
#) 

# To compile a controller, add a line like the
# following inside add_executable:
# controllers/T6TensionController.cpp
#target_link_libraries (AppSUPERballGPS ${CMAKE_CURRENT_SOURCE_DIR}/libzmq.so ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
#file(COPY "libzmq.so" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#file(COPY "libzmq.so.4" DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

#find_package(osg)
#find_package(osgViewer)
#find_package(OpenThreads)
#find_package(osgGA)
#set(OSG_LIBRARIES ${OSG_LIBRARY} ${OSGVIEWER_LIBRARY} ${OPENTHREADS_LIBRARY} ${OSGGA_LIBRARY})
#include_directories(SYSTEM ${OSG_INCLUDE_DIR})

#target_link_libraries (AppSUPERballGPS FileHelpers ${OSG_LIBRARIES})
#target_link_libraries (AppSUPERballGPS FileHelpers /opt/ros/indigo/lib/librostime.so /opt/ros/indigo/lib/libroscpp.so /opt/ros/indigo/lib/librosconsole.so /opt/ros/indigo/lib/libroscpp_serialization.so ${OSG_LIBRARIES})
