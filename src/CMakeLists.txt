cmake_minimum_required(VERSION 3.14)
project(CV05)

# add libs you need
set(OpenCV_LIBS opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs opencv_calib3d opencv_viz)
set(SRC main.cpp cube.h cube.cpp particle.h triangle.h wtriangle.cpp wtriangle.h marchingcubestables.h timer.h)

add_executable(${PROJECT_NAME} ${SRC} cli.h colors.h)
add_executable(MarchingCubeTest marchingcubetest.cpp cube.h cube.cpp particle.h cli.h colors.h triangle.h wtriangle.cpp wtriangle.h marchingcubestables.h timer.h)

find_package(OpenMP REQUIRED)

if (OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${VTK_LIBRARIES} OpenMP::OpenMP_CXX)
  target_link_libraries(MarchingCubeTest PRIVATE ${OpenCV_LIBS} ${VTK_LIBRARIES} OpenMP::OpenMP_CXX)
  #  target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} OpenMP::OpenMP_CXX)
else ()
  target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} ${VTK_LIBRARIES})
  target_link_libraries(MarchingCubeTest PRIVATE ${OpenCV_LIBS} ${VTK_LIBRARIES})
  #  target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} )
endif ()

