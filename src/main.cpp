#include <iostream>

#include <opencv2/viz/widgets.hpp>
#include <opencv2/viz/viz3d.hpp>

int main(int argc, const char **argv) {
  cv::viz::Viz3d window = cv::viz::Viz3d("Marching cubes");
  
  cv::viz::WCube mainCube(cv::Point3f(0.5,0.5,0.5), cv::Point3f(1,1,1), true, cv::viz::Color::blue());
  mainCube.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
  window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  window.showWidget("Cube widget", mainCube);
  
  window.spin();
  return 0;
}