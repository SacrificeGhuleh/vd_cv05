#include <iostream>
#include <fstream>

#include <opencv2/viz/widgets.hpp>
#include <opencv2/viz/viz3d.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <opencv2/viz/vizcore.hpp>

#include "particle.h"
#include "cube.h"
#include "wtriangle.h"

//const float step = 0.0125f;
//const float step = 0.1f;
const float step = 0.2f;
//const float step = 0.5f;
const float halfStep = (step / 2.f);
const float threshold = 0.5f;
const int maxFlannResults = 1024;
const float maxDist = sqrt((step * step) + (step / 2.f) * (step / 2.f));

cv::Mat sliceMat(cv::Mat L, int dim, std::vector<int> _sz) {
  cv::Mat M(L.dims - 1, std::vector<int>(_sz.begin() + 1, _sz.end()).data(), CV_32FC1, L.data + L.step[0] * 0);
  return M;
}

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
  
  cv::Vec3f cam_pos(2.0f, 2.0f, 2.0f);
  cv::Vec3f cam_focal_point(0.0f, 0.0f, 0.5f);
  cv::Vec3f cam_y_dir(0.0f, 0.0f, -1.0f);
  cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
  
  cv::viz::Viz3d window = cv::viz::Viz3d("Marching cubes");
  window.setWindowSize(cv::Size_(500, 500));
//  window.setViewerPose(cam_pose);
  
  cv::viz::WCube mainCube(min, max, true, cv::viz::Color::blue());
  mainCube.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
//  window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
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
  
  std::cout << "flann built" << std::endl;

//  cv::viz::WCloud myCloud(pointCloud);
//  window.showWidget("My cloud", myCloud);
  
  int sizes[] = {static_cast<int>(1.f / step) + 1, static_cast<int>(1.f / step) + 1, static_cast<int>(1.f / step) + 1};
  cv::Mat1f edgesValues(3, sizes);
  
  id = 0;
  cv::Point3f marchingMax = min + cv::Point3d(step, step, step);
  for (int x = 0; x <= 1. / step; x++) {
    marchingMax.y = min.y + step;
    for (int y = 0; y <= 1. / step; y++) {
      marchingMax.z = min.z + step;
      for (int z = 0; z <= 1. / step; z++) {
        int idx[] = {x, y, z};
        
        std::vector<int> indices;
        std::vector<float> distances;
        std::vector<float> query = {marchingMax.x, marchingMax.y, marchingMax.z};
        
        float rho = 0;
        int n = flannIndex.radiusSearch(query, indices, distances, maxDist, maxFlannResults, cv::flann::SearchParams(256));
        if (n > 0) {
          const int indicesCount = std::min<int>(indices.size(), n);
          for (int i = 0; i < indicesCount; i++) {
            const float distance = distances.at(i);
            const int indice = indices.at(i);
            const Particle &particle = particles.at(indice);
            rho += particle.rho * gaussianKernel(distance, maxDist);
          }
        }
        edgesValues.at<float>(idx) = rho;
        id++;
        marchingMax.z += step;
      }
      marchingMax.y += step;
    }
    marchingMax.x += step;
  }


//  cv::normalize(edgesValues, edgesValues, 0, 1, cv::NORM_MINMAX);
  
  std::cout << "Vertices sampled" << std::endl;

//  std::vector<int> vectSizes = {static_cast<int>(1.f/step)+1,static_cast<int>(1.f/step)+1,static_cast<int>(1.f/step)+1};
//
//  for(int i = 0; i < vectSizes.at(0); i++){
//    std::cout << sliceMat(edgesValues, i, vectSizes) << std::endl;
//  }
  
  std::vector<Triangle> triangles;
  
  marchingMax = min + cv::Point3d(step, step, step);
  id = 0;
  for (int x = 0; x < 1. / step; x++) {
    marchingMax.y = min.y + step;
    for (int y = 0; y < 1. / step; y++) {
      marchingMax.z = min.z + step;
      for (int z = 0; z < 1. / step; z++) {
        std::stringstream ss;
        ss << "Sphere " << id;
        
        Cube cube(marchingMax);
        
        int pts[8][3] = {
            {x + 1 - 1, y + 1 - 1, y + 1 - 1},
            {x + 1 - 1, y + 1 - 0, y + 1 - 1},
            {x + 1 - 0, y + 1 - 0, y + 1 - 1},
            {x + 1 - 0, y + 1 - 1, y + 1 - 1},
            {x + 1 - 1, y + 1 - 1, y + 1 - 0},
            {x + 1 - 1, y + 1 - 0, y + 1 - 0},
            {x + 1 - 0, y + 1 - 0, y + 1 - 0},
            {x + 1 - 0, y + 1 - 1, y + 1 - 0}
        };
        
        for (int i = 0; i < 8; i++) {
          cube.values.at(i) = edgesValues.at<float>(pts[i]);
        }
        
        cube.generateTriangles(triangles);
        
        cube.draw(window, id);
        
        id++;
        marchingMax.z += step;
      }
      marchingMax.y += step;
    }
    marchingMax.x += step;
  }
  
  std::cout << "Cubes generated" << std::endl;
  
  
  int idx = 0;
  for (const Triangle &t : triangles) {
    std::stringstream ss;
    ss << "Triangle " << idx++;
    window.showWidget(ss.str(), cv::viz::WTriangle(t));
  }
  
  window.spin();
  return 0;
}