link_libraries(tgcreator controllers core)

add_executable(AppRodSpringTest
    T1Model.cpp
    AppRodSpringTest.cpp
) 

# To compile a controller, add a line like the
# following inside add_executable:
#    controllers/T6TensionController.cpp
