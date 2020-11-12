//
// Created by richardzvonek on 11/12/20.
//

#ifndef VD_CV04_PARTICLE_H
#define VD_CV04_PARTICLE_H

#include <half.hpp>

struct Particle {
  half_float::half position_x; // particle position (m)
  half_float::half position_y;
  half_float::half position_z;
  
  half_float::half velocity_x; // particle velocity (m/s)
  half_float::half velocity_y;
  half_float::half velocity_z;
  
  half_float::half rho; // density (kg/m3)
  half_float::half pressure;
  half_float::half radius; // particle radius (m)
};


#endif //VD_CV04_PARTICLE_H
