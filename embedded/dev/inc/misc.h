#ifndef _MISC_H_
#define _MISC_H_
#include <math.h>

inline float vector_norm(const float* const v, const uint8_t length)
{
  uint8_t i;
  float norm = 0.0f;
  for (i = 0; i < length; i++)
    norm += v[i]*v[i];
  return sqrt(norm);
}

inline void vector_normalize(float* const v, const uint8_t length)
{
  uint8_t i;
  float norm = vector_norm(v, length);
  for (i = 0; i < length; i++)
    v[i] /= norm;
}

inline void vector3_cross(const float* const a, const float* const b,
  float* const result)
{
  result[0] = a[1]*b[2] - a[2]*b[1];
  result[1] = a[2]*b[0] - a[0]*b[2];
  result[2] = a[0]*b[1] - a[1]*b[0];
}

inline void q_derivative(const float* const q, const float* const v,
  float* const dq)
{
  dq[0] = 0.5 * (v[0] * -q[1] + v[1] * -q[2] + v[2] * -q[3]);
  dq[1] = 0.5 * (v[0] *  q[1] + v[1] * -q[2] + v[2] *  q[3]);
  dq[2] = 0.5 * (v[0] *  q[1] + v[1] *  q[2] + v[2] * -q[3]);
  dq[3] = 0.5 * (v[0] * -q[1] + v[1] *  q[2] + v[2] *  q[3]);
}

#endif
