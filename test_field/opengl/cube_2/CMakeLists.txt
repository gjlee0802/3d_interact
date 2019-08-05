cmake_minimum_required(VERSION 2.8.12)

project(qtgl)

# Qt library
find_package(Qt5Widgets)
find_package(Qt5OpenGL)

# OpenGL & GLEW library
find_package(OpenGL)

# executable
add_executable(qt5 qtgl.cpp glwidget.h)
qt5_use_modules(qt5 Widgets OpenGL)
target_link_libraries(qt5 ${QT_LIBRARIES} ${OPENGL_opengl_LIBRARY})

#add_executable(gl cube.cpp)
#target_link_libraries(gl ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})
