#include <iostream>
#include <fstream>

#include <opencv2/viz/widgets.hpp>
#include <opencv2/viz/viz3d.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <opencv2/viz/vizcore.hpp>

#include "particle.h"
#include "cube.h"

//const float step = 0.0125f;
const float step = 0.1f;
//const float step = 0.2f;
const float halfStep = (step / 2.f);
const float threshold = 600;
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
  
  cv::Vec3f cam_pos(2.0f,2.0f,2.0f);
  cv::Vec3f cam_focal_point(0.0f,0.0f,0.5f);
  cv::Vec3f cam_y_dir(0.0f,0.0f,-1.0f);
  cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
  
  
  cv::viz::Viz3d window = cv::viz::Viz3d("Marching cubes");
  window.setWindowSize(cv::Size_(500,500));
  window.setViewerPose(cam_pose);
  
  cv::viz::WCube mainCube(min, max, true, cv::viz::Color::blue());
  mainCube.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
  window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  window.showWidget("Cube widget", mainCube);
  
  std::vector<Particle> particles;
  loadData(particles, "./data/sph_001100.bin");
  
  cv::Mat3f pointCloud(1, 64000);
  
  auto samplePoints = cv::Mat1f(64000, 3);
  int id = 0;
  for (const Particle &particle : particles) {
    pointCloud.at<cv::Vec3f>(id) = cv::Vec3f(particle.position_x, particle.position_y, particle.position_z);

    samplePoints.at<float>(id, 0) = particle.position_x;
    samplePoints.at<float>(id, 1) = particle.position_y;
    samplePoints.at<float>(id, 2) = particle.position_z;
    
    id++;
  }

  cv::flann::Index flannIndex = cv::flann::Index(samplePoints, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_L2);
  
  cv::viz::WCloud myCloud(pointCloud);
  window.showWidget("My cloud", myCloud);
  
  int sizes[] = {static_cast<int>(1.f/step)+1,static_cast<int>(1.f/step)+1,static_cast<int>(1.f/step)+1};
  cv::Mat1f edgesValues(3, sizes);
  cv::Mat_<bool> alreadyInitialized(3, sizes, false);
  
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
          
          std::vector<int> indices;
          std::vector<float> distances;

          for (int j = 0; j < 8; j++) {
            const cv::Point3f &pointInCube = cube.points.at(j);
            std::vector<float> query = {pointInCube.x, pointInCube.y, pointInCube.z};
            float rho = 0;
            int n = flannIndex.radiusSearch(query, indices, distances, maxDist, 10 * 1024, cv::flann::SearchParams(256));
            if (n > 0) {
              const int indicesCount = std::min<int>(indices.size(), n);
              for (int i = 0; i < indicesCount; i++) {
                const float distance = distances.at(i);
                const int indice = indices.at(i);
                const Particle &particle = particles.at(indice);
                rho += particle.rho * gaussianKernel(distance, maxDist);
              }
            }
            cube.values[j] = rho;
            if (rho > threshold) {
              std::stringstream ssPt;
              ssPt << "Point " << j;

              window.showWidget(ssPt.str(), cv::viz::WSphere(pointInCube, step / 20.f));
            }
            j++;
          }
          
//          window.resetCamera();
          auto cam  = window.getCamera();
          window.spinOnce(10, true);
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