//
// Created by richardzvonek on 11/12/20.
//
#ifndef VD_CV04_CUBE_H
#define VD_CV04_CUBE_H

#include <opencv2/core/types.hpp>
#include <opencv2/viz/viz3d.hpp>



extern const float halfStep;

class Cube {
public:
  explicit Cube(const cv::Point3f &max);
  
  std::array<cv::Point3f, 8> points;
  std::array<float, 8> values;
  cv::Point3f center;
  
  inline cv::Point3f getMax() const {
    return points[1];
  }
  
  inline cv::Point3f getMin() const {
    return points[7];
  }
  
};


#endif //VD_CV04_CUBE_H
