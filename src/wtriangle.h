//
// Created by richardzvonek on 11/12/20.
//

#ifndef VD_CV04_WTRIANGLE_H
#define VD_CV04_WTRIANGLE_H


#include <opencv2/viz/widgets.hpp>
#include <opencv2/core/types.hpp>
#include "triangle.h"

namespace cv {
  namespace viz {
    class WTriangle : public Widget3D {
    public:
      explicit WTriangle(const Triangle& t) : WTriangle(t.v1, t.v2, t.v3){};
      
      WTriangle(const Point3f &pt1, const Point3f &pt2, const Point3f &pt3, const viz::Color &color = viz::Color::white());
    };
    
  };
};


#endif //VD_CV04_WTRIANGLE_H
