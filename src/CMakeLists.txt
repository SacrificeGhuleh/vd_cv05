CMAKE_MINIMUM_REQUIRED(VERSION 3.14)
PROJECT(CV05)

# add libs you need
SET(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_calib3d opencv_viz)
SET(SRC main.cpp cube.h cube.cpp particle.h)

ADD_EXECUTABLE(${PROJECT_NAME} ${SRC} cli.h colors.h)

find_package(OpenMP REQUIRED)

if (OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${VTK_LIBRARIES} OpenMP::OpenMP_CXX)
#  target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} OpenMP::OpenMP_CXX)
else ()
  target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${VTK_LIBRARIES})
#  target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} )
endif ()
