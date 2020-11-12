#include <iostream>
#include <fstream>

#include <opencv2/viz/widgets.hpp>
#include <opencv2/viz/viz3d.hpp>
#include <opencv2/viz/vizcore.hpp>

#include "particle.h"
#include "cube.h"
#include "triangle.h"
#include "wtriangle.h"

//const float step = 0.0125f;
const float step = 0.1f;
//const float step = 0.2f;
const float halfStep = (step / 2.f);
const float threshold = 600;
const int maxFlannResults = 1024;
const float maxDist = sqrt((step * step) + (step / 2.f) * (step / 2.f));

void loadData(std::vector<Particle> &particles, const std::string &filename) {
  std::ifstream file;
  file.open(filename, std::ios::binary);
  
  if (file.is_open()) {
    
    file.seekg(0, std::ifstream::end);
    size_t length = file.tellg();
    file.seekg(0, std::ifstream::beg);
    
    size_t numberOfParticles = length / sizeof(Particle);
    
    
    std::cout << "Length: " << length << std::endl;
    std::cout << "Number of particles: " << numberOfParticles << std::endl;
    
    particles.resize(numberOfParticles);
    
    if (file.read(reinterpret_cast<char *>(particles.data()), length)) {
      std::cout << "reading ok" << std::endl;
    } else {
      std::cout << "Unable to read from file " << std::endl;
    }
    
  } else {
    std::cout << "Unable to open file " << std::endl;
  }
  file.close();
}

template<class T>
T gaussianKernel(const T &r, const T &h) {
  T ret = std::cbrt(M_PI) * (h * h * h);
  ret = 1.f / ret;
  ret *= std::exp(-((r / h) * (r / h)));
  return ret;
}

int main(int argc, const char **argv) {
  const cv::Point3d min(-0.5f, -0.5f, 0.f);
  const cv::Point3d max(0.5f, 0.5f, 1.f);
  
//  cv::Vec3f cam_pos(2.0f,2.0f,2.0f);
//  cv::Vec3f cam_focal_point(0.0f,0.0f,0.5f);
//  cv::Vec3f cam_y_dir(0.0f,0.0f,-1.0f);
//  cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
  
  
  cv::viz::Viz3d window = cv::viz::Viz3d("Marching cubes");
  window.setWindowSize(cv::Size_(500,500));
//  window.setViewerPose(cam_pose);
  
  Cube cube(cv::Point3f(0, 0, 0));
  cube.values[0] = threshold * 2;
  cube.values[1] = threshold * 2;
  cube.draw(window, 0);
  
  std::vector<Triangle> triangles;
  cube.generateTriangles(triangles);
  int idx = 0;
  for (const Triangle& t : triangles) {
    std::stringstream ss;
    ss << "Triangle" << idx++;
  
    cv::viz::WTriangle wtriangle(t);
    window.showWidget(ss.str(), wtriangle);
  }
  
  window.spin();
  return 0;
}