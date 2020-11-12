//
// Created by richardzvonek on 11/12/20.
//

#include <opencv2/viz/widgets.hpp>

#include "cube.h"
#include "marchingcubestables.h"


cv::Point3f lerp(const cv::Point3f &p1, const cv::Point3f &p2, float d1, float d2) {
//  if (std::abs(d1) < 0.000001) return p1;
//  if (std::abs(d2) < 0.000001) return p2;
//  if (std::abs(d1 - d2) < 0.000001) return p1;
//
//  return p1 + (-1 * d1) * (p2 - p1) / (d2 - d1);

  return (p1+p2)/2;
}


Cube::Cube(const cv::Point3f &max) : center(static_cast<float>(max.x - halfStep),
                                            static_cast<float>(max.y - halfStep),
                                            static_cast<float>(max.z - halfStep)) {
  cv::Point3f p0(center.x + halfStep, center.y - halfStep, center.z + halfStep);
  cv::Point3f p1(center.x + halfStep, center.y + halfStep, center.z + halfStep);
  cv::Point3f p2(center.x - halfStep, center.y + halfStep, center.z + halfStep);
  cv::Point3f p3(center.x - halfStep, center.y - halfStep, center.z + halfStep);
  cv::Point3f p4(center.x + halfStep, center.y - halfStep, center.z - halfStep);
  cv::Point3f p5(center.x + halfStep, center.y + halfStep, center.z - halfStep);
  cv::Point3f p6(center.x - halfStep, center.y + halfStep, center.z - halfStep);
  cv::Point3f p7(center.x - halfStep, center.y - halfStep, center.z - halfStep);
  
  points = {p0, p1, p2, p3, p4, p5, p6, p7};
  values = {0, 0, 0, 0, 0, 0, 0, 0};
}


void Cube::draw(cv::viz::Viz3d &window, const int id) {
  cv::viz::WCube marchingCube(getMin(), getMax(), true, cv::viz::Color::blue());
  
  std::stringstream ss;
  ss << "Cube " << id;
  window.showWidget(ss.str(), marchingCube);
  
  ss << " Vertex";
  for (int i = 0; i < 8; i++) {
    std::stringstream ssid;
    ssid << ss.str();
    ssid << i;
    
    if (values[i] > threshold) {
      window.showWidget(ssid.str(), cv::viz::WSphere(points[i], step / 20.f));
    }
  }
}

void Cube::generateTriangles(std::vector<Triangle> &triangles) {
  
  int idx = 0;
  
  if (values[0] > threshold) idx |= 1;
  if (values[4] > threshold) idx |= 2;
  if (values[5] > threshold) idx |= 4;
  if (values[1] > threshold) idx |= 8;
  if (values[2] > threshold) idx |= 16;
  if (values[6] > threshold) idx |= 32;
  if (values[7] > threshold) idx |= 64;
  if (values[3] > threshold) idx |= 128;
  
  int numTriangles = 0;
  for (int i = 0; triTable[idx][i] != -1; i += 3) numTriangles++;
  
  cv::Point3f triVertices[12];
  
  if (edgeTable[idx] & 1)
    triVertices[0] = lerp(points[0], points[4], values[0], values[4]);
  if (edgeTable[idx] & 2)
    triVertices[1] = lerp(points[4], points[5], values[4], values[5]);
  if (edgeTable[idx] & 4)
    triVertices[2] = lerp(points[5], points[1], values[5], values[1]);
  if (edgeTable[idx] & 8)
    triVertices[3] = lerp(points[1], points[0], values[1], values[0]);
  if (edgeTable[idx] & 16)
    triVertices[4] = lerp(points[2], points[6], values[2], values[6]);
  if (edgeTable[idx] & 32)
    triVertices[5] = lerp(points[6], points[7], values[6], values[7]);
  if (edgeTable[idx] & 64)
    triVertices[6] = lerp(points[7], points[3], values[7], values[3]);
  if (edgeTable[idx] & 128)
    triVertices[7] = lerp(points[3], points[2], values[3], values[2]);
  if (edgeTable[idx] & 256)
    triVertices[8] = lerp(points[0], points[2], values[0], values[2]);
  if (edgeTable[idx] & 512)
    triVertices[9] = lerp(points[4], points[6], values[4], values[6]);
  if (edgeTable[idx] & 1024)
    triVertices[10] = lerp(points[5], points[7], values[5], values[7]);
  if (edgeTable[idx] & 2048)
    triVertices[11] = lerp(points[1], points[3], values[1], values[3]);
  
  triangles.resize(numTriangles);
  for (int i = 0; triTable[idx][i] != -1; i += 3) {
    triangles.at(i / 3) = Triangle(triVertices[triTable[idx][i]], triVertices[triTable[idx][i + 1]], triVertices[triTable[idx][i + 2]]);
    
  }
}