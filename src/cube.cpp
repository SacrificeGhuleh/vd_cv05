//
// Created by richardzvonek on 11/12/20.
//

#include "cube.h"

#include <opencv2/viz/widgets.hpp>

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
  values = {0,0,0,0,0,0,0,0};
}
