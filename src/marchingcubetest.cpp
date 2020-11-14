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
const float step = .9f;
//const float step = 0.2f;
const float halfStep = (step / 2.f);
const float threshold = 0.5;
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

void generateCubes(std::vector<Cube> &cubes) {
  // Output should match with https://www.researchgate.net/figure/Basic-cases-in-the-Marching-Cubes-algorithm-and-examples-of-how-the-points-of_fig1_3410984
  
  cv::Point3f pt(0, 0, 0);
  Cube c0(pt);
  c0.values = {0, 0, 0, 0, 0, 0, 0, 0};
  pt.y++;
  
  Cube c1(pt);
  c1.values = {0, 0, 0, 1, 0, 0, 0, 0};
  pt.y++;
  
  Cube c2(pt);
  c2.values = {0, 0, 1, 1, 0, 0, 0, 0};
  pt.y++;
  
  Cube c3(pt);
  c3.values = {0, 0, 0, 1, 0, 0, 1, 0};
  pt.y++;
  
  Cube c4(pt);
  c4.values = {0, 0, 0, 1, 0, 1, 0, 0};
  pt.y = 0;
  pt.z++;
  
  Cube c5(pt);
  c5.values = {1, 1, 1, 0, 0, 0, 0, 0};
  pt.y++;
  
  Cube c6(pt);
  c6.values = {0, 0, 1, 1, 0, 1, 0, 0};
  pt.y++;
  
  Cube c7(pt);
  c7.values = {0, 0, 1, 0, 0, 1, 0, 1};
  pt.y++;
  
  Cube c8(pt);
  c8.values = {1, 1, 1, 1, 0, 0, 0, 0};
  pt.y++;
  
  Cube c9(pt);
  c9.values = {1, 1, 0, 1, 1, 0, 0, 0};
  pt.y = 0;
  pt.z++;
  
  Cube c10(pt);
  c10.values = {0, 1, 0, 1, 0, 1, 0, 1};
  pt.y++;
  
  Cube c11(pt);
  c11.values = {1, 1, 0, 1, 0, 1, 0, 0};
  pt.y++;
  
  Cube c12(pt);
  c12.values = {1, 1, 1, 0, 0, 0, 0, 1};
  pt.y++;
  
  
  Cube c13(pt);
  c13.values = {0, 1, 0, 1, 1, 0, 1, 0};
  pt.y++;
  
  cubes.emplace_back(c0);
  cubes.emplace_back(c1);
  cubes.emplace_back(c2);
  cubes.emplace_back(c3);
  cubes.emplace_back(c4);
  cubes.emplace_back(c5);
  cubes.emplace_back(c6);
  cubes.emplace_back(c7);
  cubes.emplace_back(c8);
  cubes.emplace_back(c9);
  cubes.emplace_back(c10);
  cubes.emplace_back(c11);
  cubes.emplace_back(c12);
  cubes.emplace_back(c13);
}

int main(int argc, const char **argv) {
  cv::viz::Viz3d window = cv::viz::Viz3d("Marching cubes");
  window.setWindowSize(cv::Size_(500, 500));
  
  cv::Vec3f cam_pos(10.0f, 1.5f, 1.0f);
  cv::Vec3f cam_focal_point(0.0f, 1.5f, 1.f);
  cv::Vec3f cam_y_dir(0.0f, 0.0f, -1.0f);
  cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
  window.setViewerPose(cam_pose);
  
  window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  
  std::vector<Cube> cubes;
  generateCubes(cubes);
  
  int cubeIdx = 0;
  for (Cube &c : cubes) {
    c.draw(window, cubeIdx);
    std::vector<Triangle> triangles;
    c.generateTriangles(triangles);
    int idx = 0;
    for (const Triangle &t : triangles) {
      std::stringstream ss;
      ss << cubeIdx << "Triangle" << idx++;
      
      cv::viz::WTriangle wtriangle(t);
      window.showWidget(ss.str(), wtriangle);
//      window.spinOnce(1, true);
    }
    cubeIdx++;
  }


//  Cube cube(cv::Point3f(1, 1, 1));
//  window.spinOnce(100, true);
////  cube.values[0] = threshold * 2;
////  cube.values[1] = threshold * 2;
////  cube.values[2] = threshold * 2;
////  cube.values[3] = threshold * 2;
//  cube.values[4] = threshold * 2;
////  cube.values[5] = threshold * 2;
////  cube.values[6] = threshold * 2;
////  cube.values[7] = threshold * 2;
//
//  cube.draw(window, 0);
//
//  std::vector<Triangle> triangles;
//  cube.generateTriangles(triangles);
//  int idx = 0;
//  for (const Triangle &t : triangles) {
//    std::stringstream ss;
//    ss << "Triangle" << idx++;
//
//    cv::viz::WTriangle wtriangle(t);
//    window.showWidget(ss.str(), wtriangle);
//  }
  
  window.spin();
  return 0;
}