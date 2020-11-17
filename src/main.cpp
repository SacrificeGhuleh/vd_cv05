#include <iostream>
#include <fstream>
#include <unordered_map>

#include <opencv2/viz/widgets.hpp>
#include <opencv2/viz/viz3d.hpp>
#include <opencv2/flann/miniflann.hpp>
#include <opencv2/viz/vizcore.hpp>

#include "particle.h"
#include "cube.h"
#include "wtriangle.h"
#include "timer.h"
#include "wsurface.h"

//const float step = 0.00625f;
//const float step = 0.0125f;
const float step = 0.025f;
//const float step = 0.05f;
//const float step = 0.1f;
//const float step = 0.2f;
//const float step = 0.5f;
const float halfStep = (step / 2.f);
const float threshold = 600.f;
const int maxFlannResults = 128;
const float maxDist = sqrt((step * step) + (step / 2.f) * (step / 2.f));

const float smoothingLength = 0.055f;

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


template<class T>
T cubicSplineKernel(const T &r, const T &h) {
  T q = r / h;
  T ret;
  if (q < 1 && q >= 0) {
    ret = (2. / 3.) - (q * q) + ((1. / 2.) * (q * q * q));
  } else if (q < 2 && q >= 1) {
    ret = 2. - q;
    ret = ret * ret * ret;
    ret *= (1. / 6.);
  } else {
    ret = 0;
  }
  return (3. / (2. * M_PI * (h * h * h))) * ret;
}

struct Point3fHash {
  size_t operator()(const cv::Point3f &x) const noexcept {
    size_t seed = 0;
    constexpr uint32_t GOLDEN_RATIO = 0x9e3779b9;
    seed ^= std::hash<float>()(x.x) + GOLDEN_RATIO + (seed << 6) + (seed >> 2);
    seed ^= std::hash<float>()(x.y) + GOLDEN_RATIO + (seed << 6) + (seed >> 2);
    seed ^= std::hash<float>()(x.z) + GOLDEN_RATIO + (seed << 6) + (seed >> 2);
    
    return seed;
  }
};

int main(int argc, const char **argv) {
  std::string filename;
  if (argc == 1) {
    filename = "data/sph_001100.bin";
  } else {
    filename = argv[1];
  }
  
  std::string outFileName = "out/" + filename + ".jpg";
  const cv::Point3d min(-0.5f, -0.5f, 0.f);
  const cv::Point3d max(0.5f, 0.5f, 1.f);
  
  cv::Vec3f cam_pos(2.0f, 2.0f, 2.0f);
  cv::Vec3f cam_focal_point(0.0f, 0.0f, 0.5f);
  cv::Vec3f cam_y_dir(0.0f, 0.0f, -1.0f);
  cv::Affine3f cam_pose = cv::viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
  
  cv::viz::Viz3d window = cv::viz::Viz3d("Marching cubes");
  window.setWindowSize(cv::Size_(500, 500));
  window.setViewerPose(cam_pose);
  
  cv::viz::WCube mainCube(min, max, true, cv::viz::Color::blue());
  mainCube.setRenderingProperty(cv::viz::LINE_WIDTH, 4.0);
//  window.showWidget("Coordinate Widget", cv::viz::WCoordinateSystem());
  window.showWidget("Cube widget", mainCube);
  
  std::vector<Particle> particles;
  Timer loadDataTimer;
  loadData(particles, filename);
  double loadTime = loadDataTimer.elapsed();
  
  Timer flannTimer;
//  cv::Mat3f pointCloud(1, 64000);
  
  auto samplePoints = cv::Mat1f(64000, 3);
  int id = 0;
  for (const Particle &particle : particles) {
//    pointCloud.at<cv::Vec3f>(id) = cv::Vec3f(particle.position_x, particle.position_y, particle.position_z);
    
    samplePoints.at<float>(id, 0) = particle.position_x;
    samplePoints.at<float>(id, 1) = particle.position_y;
    samplePoints.at<float>(id, 2) = particle.position_z;
    
    id++;
  }
  
  cv::flann::Index flannIndex = cv::flann::Index(samplePoints, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_L2);
  double flannTime = flannTimer.elapsed();
  std::cout << "flann built" << std::endl;

//  cv::viz::WCloud myCloud(pointCloud);
//  window.showWidget("My cloud", myCloud);
  
  int sizes[] = {static_cast<int>(1.f / step) + 1, static_cast<int>(1.f / step) + 1, static_cast<int>(1.f / step) + 1};
  cv::Mat1f edgesValues(3, sizes);
  
  
  std::vector<Triangle> triangles;
  std::unordered_map<cv::Point3f, float, Point3fHash> valuesMap;
  
  Timer marchingTimer;
  
  size_t unorderedMapHits = 0;
  size_t unorderedMapNotHits = 0;
  cv::Point3f marchingMax = min + cv::Point3d(step, step, step);
  id = 0;
  const int iterations = static_cast<int>(1. / step);
  for (int x = 0; x < iterations; x++) {
    marchingMax.y = min.y + step;
    for (int y = 0; y < iterations; y++) {
      marchingMax.z = min.z + step;
      for (int z = 0; z < iterations; z++) {
        std::stringstream ss;
        ss << "Sphere " << id;
        
        Cube cube(marchingMax);
        
        for (int ptIdx = 0; ptIdx < 8; ptIdx++) {
          const cv::Point3f &pt = cube.points.at(ptIdx);
          const auto it = valuesMap.find(pt);
          
          if (it != valuesMap.end()) {
            unorderedMapHits++;
            cube.values.at(ptIdx) = it->second;
          } else {
            unorderedMapNotHits++;
            std::vector<int> indices;
            std::vector<float> distances;
            std::vector<float> query = {pt.x, pt.y, pt.z};
            
            float rho = 0;
            int n = flannIndex.radiusSearch(query, indices, distances, maxDist, maxFlannResults, cv::flann::SearchParams(256));
            if (n > 0) {
              const int indicesCount = std::min<int>(indices.size(), n);
              for (int indiceIdx = 0; indiceIdx < indicesCount; indiceIdx++) {
                const float distance = distances.at(indiceIdx);
                const int indice = indices.at(indiceIdx);
                const Particle &particle = particles.at(indice);
//                rho += particle.rho * cubicSplineKernel(distance, smoothingLength);
                rho += particle.getMass() * cubicSplineKernel(distance, smoothingLength);
              }
            }
            cube.values.at(ptIdx) = rho;
            valuesMap.insert(std::make_pair(pt, rho));
          }
        }
        cube.generateTriangles(triangles);
//        cube.draw(window, id);
        
        id++;
        marchingMax.z += step;
      }
      marchingMax.y += step;
    }
    std::cout << "computed " << static_cast<float>(x) / static_cast<float>(iterations) * 100.f << "%\n";
    marchingMax.x += step;
  }
  double marchingTime = marchingTimer.elapsed();
  
  std::cout << "Unordered map reused: " << unorderedMapHits << std::endl;
  std::cout << "Unordered map unique: " << unorderedMapNotHits << std::endl;
  
  std::cout << "Cubes generated" << std::endl;
  
  Timer renderTimer;
  
  window.showWidget("MySurface", cv::viz::WSurface(triangles));
  
  window.spinOnce(1, true);
  window.saveScreenshot(outFileName);
//  window.spin();
  
  double renderTime = renderTimer.elapsed();
  
  std::cout << "Marching cubes benchmark\n";
  std::cout << "      Step | " << step << "\n";
  std::cout << "---------- |\n";
  std::cout << "   Loading | " << loadTime << "\n";
  std::cout << "     Flann | " << flannTime << "\n";
  std::cout << "  Marching | " << marchingTime << "\n";
  std::cout << "    Render | " << renderTime << "\n";
  
  
  return 0;
}