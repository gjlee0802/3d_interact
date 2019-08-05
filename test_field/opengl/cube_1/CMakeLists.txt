CMAKE_MINIMUM_REQUIRED(VERSION 2.8.8)

project(gl)

find_package(OpenGL)
find_package(GLUT)

include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

add_executable(cube_turn cube_turn.cpp)
target_link_libraries(cube_turn ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})

#add_executable(gl cube.cpp)
#target_link_libraries(gl ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
