//
// Created by richardzvonek on 11/12/20.
//

#ifndef VD_CV04_TRIANGLE_H
#define VD_CV04_TRIANGLE_H

#include <opencv2/core/types.hpp>

struct Triangle {
  Triangle() : v1(0, 0, 0), v2(0, 0, 0), v3(0, 0, 0) {}
  
  Triangle(const cv::Point3f &p1, const cv::Point3f &p2, const cv::Point3f &p3) : v1(p1), v2(p2), v3(p3) {}
  
  Triangle(const Triangle &t) : v1(t.v1), v2(t.v2), v3(t.v3) {}
  
  cv::Point3f v1;
  cv::Point3f v2;
  cv::Point3f v3;
};
#endif //VD_CV04_TRIANGLE_H
