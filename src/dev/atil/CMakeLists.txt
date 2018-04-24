

project(atil)

file(GLOB_RECURSE SRC_ATIL ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

add_library( ${PROJECT_NAME} SHARED ${SRC_ATIL} )
target_link_libraries(${PROJECT_NAME} tgcreator controllers core)

link_libraries(tgcreator core Adapters)

add_executable(AppSuperBallLearning
    SuperBallModel.cpp
    SuperBallLearningController.cpp
    AppSUPERball.cpp
    heightSensor.cpp
) 
