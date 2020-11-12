#include <iostream>
#include <fstream>

#include <opencv2/viz/widgets.hpp>
#include <opencv2/viz/viz3d.hpp>

#include "particle.h"
#include "cube.h"

//const float step = 0.0125f;
const float step = 0.1f;
//const float step = 0.2f;

const float halfStep = (step / 2.f);

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

int main(int argc, const char **argv) {
  const cv::Point3d min(-0.5f, -0.5f, 0.f);
  const cv::Point3d max(0.5f, 0.5f, 1.f);
  
  
  cv::viz::Viz3d window = cv::viz::Viz3d("Marching cubes");
  
  cv::viz::WCube mainCube(min, max, true, cv::viz::Color::blue());
  mainCube.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
  window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  window.showWidget("Cube widget", mainCube);
  
  std::vector<Particle> particles;
  loadData(particles, "./data/sph_001100.bin");
  
  cv::Mat3f pointCloud(1, 64000);
//  cv::Mat3b pointCloudColors(1, 64000);
  
  int id = 0;
  for (const Particle &particle : particles) {
    pointCloud.at<cv::Vec3f>(id) = cv::Vec3f(particle.position_x, particle.position_y, particle.position_z);
//    pointCloudColors.at<cv::Vec3b>(id) = colors[id%colorsSize];
    
    id++;
  }
  cv::viz::WCloud myCloud(pointCloud);
  window.showWidget("My cloud", myCloud);
  
  while (!window.wasStopped()) {
    cv::Point3d marchingMax = min + cv::Point3d(step, step, step);
    id = 0;
    
    for (int x = 0; x < 1. / step; x++) {
      marchingMax.y = min.y + step;
      for (int y = 0; y < 1. / step; y++) {
        marchingMax.z = min.z + step;
        for (int z = 0; z < 1. / step; z++) {
          std::stringstream ss;
          ss << "Cube " << id;
          
          Cube cube(marchingMax);
          
          window.showWidget("Cube widget", mainCube);
          
          cv::viz::WCube marchingCube(cube.getMin(), cube.getMax(), true, cv::viz::Color::blue());
          window.showWidget(ss.str(), marchingCube);
          window.showWidget("My cloud", myCloud);
          
          window.spinOnce(1, true);
          window.removeAllWidgets();
          
          id++;
          marchingMax.z += step;
        }
        marchingMax.y += step;
      }
      marchingMax.x += step;
    }
  }
  window.showWidget("My cloud", myCloud);
  
  
  window.spin();
  return 0;
}