//
// Created by richardzvonek on 11/12/20.
//

#include <opencv2/viz/widgets.hpp>
#include <opencv2/highgui.hpp>

#include "cube.h"
#include "marchingcubestables.h"


cv::Point3f lerp(const cv::Point3f &p1, const cv::Point3f &p2, float d1, float d2) {
  float t = (threshold - d1) / (d2 - d1);
  return p1 + t * (p2 - p1);
//
//  if (std::abs(d1) < 0.000001) return p1;
//  if (std::abs(d2) < 0.000001) return p2;
//  if (std::abs(d1 - d2) < 0.000001) return p1;
//
  return p1 + (-1 * d1) * (p2 - p1) / (d2 - d1);
}


Cube::Cube(const cv::Point3f &max) : center(static_cast<float>(max.x - halfStep),
                                            static_cast<float>(max.y - halfStep),
                                            static_cast<float>(max.z - halfStep)) {
  cv::Point3f p0(center.x - halfStep, center.y - halfStep, center.z - halfStep);
  cv::Point3f p1(center.x - halfStep, center.y + halfStep, center.z - halfStep);
  cv::Point3f p2(center.x + halfStep, center.y + halfStep, center.z - halfStep);
  cv::Point3f p3(center.x + halfStep, center.y - halfStep, center.z - halfStep);
  cv::Point3f p4(center.x - halfStep, center.y - halfStep, center.z + halfStep);
  cv::Point3f p5(center.x - halfStep, center.y + halfStep, center.z + halfStep);
  cv::Point3f p6(center.x + halfStep, center.y + halfStep, center.z + halfStep);
  cv::Point3f p7(center.x + halfStep, center.y - halfStep, center.z + halfStep);
  
  points = {p0, p1, p2, p3, p4, p5, p6, p7};
  values = {0, 0, 0, 0, 0, 0, 0, 0};
}


void Cube::draw(cv::viz::Viz3d &window, const int id) {
  cv::viz::WCube marchingCube(getMin(), getMax(), true, cv::viz::Color::blue());
  
  std::stringstream ss;
  ss << "Cube " << id;
  window.showWidget(ss.str(), marchingCube);
//  window.spinOnce(1,true);
  
  ss << " Vertex";
  for (int i = 0; i < 8; i++) {
    if (values[i] > threshold) {
      std::stringstream ssid;
      ssid << ss.str() << i;
      
      window.showWidget(ssid.str(), cv::viz::WSphere(points[i], step / 20.f, 10, cv::viz::Color(values[i])));
//    cv::waitKey(1000);
//    window.spinOnce(1,true);
    
    }
  }
}

void Cube::generateTriangles(std::vector<Triangle> &triangles) {
  
  unsigned int idx = 0;
  
  if (values[0] > threshold) idx |= 1;
  if (values[1] > threshold) idx |= 2;
  if (values[2] > threshold) idx |= 4;
  if (values[3] > threshold) idx |= 8;
  if (values[4] > threshold) idx |= 16;
  if (values[5] > threshold) idx |= 32;
  if (values[6] > threshold) idx |= 64;
  if (values[7] > threshold) idx |= 128;

//  int numTriangles = 0;
//  for (int i = 0; triTable[idx][i] != -1; i += 3) numTriangles++;
  
  for (int i = 0; triTable[idx][i] != -1; i += 3) {
    unsigned int a0 = cornerIndexAFromEdge[triTable[idx][i]];
    unsigned int b0 = cornerIndexBFromEdge[triTable[idx][i]];
    
    unsigned int a1 = cornerIndexAFromEdge[triTable[idx][i + 1]];
    unsigned int b1 = cornerIndexBFromEdge[triTable[idx][i + 1]];
    
    unsigned int a2 = cornerIndexAFromEdge[triTable[idx][i + 2]];
    unsigned int b2 = cornerIndexBFromEdge[triTable[idx][i + 2]];
    
    Triangle triangle;
    
    triangle.v1 = lerp(points[a0], points[b0], values[a0], values[b0]);
    triangle.v2 = lerp(points[a1], points[b1], values[a1], values[b1]);
    triangle.v3 = lerp(points[a2], points[b2], values[a2], values[b2]);
    triangles.emplace_back(triangle);
  }
}