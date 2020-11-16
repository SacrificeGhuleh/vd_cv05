//
// Created by richardzvonek on 11/16/20.
//

#ifndef VD_CV04_WSURFACE_H
#define VD_CV04_WSURFACE_H

#include <opencv2/viz/widgets.hpp>

#include <vector>
#include "triangle.h"

namespace cv {
  namespace viz {
    class WSurface  : public Widget3D{
    public:
      explicit WSurface(const std::vector<Triangle>& triangles, const viz::Color &color = viz::Color::white());
    };
  }}


#endif //VD_CV04_WSURFACE_H
