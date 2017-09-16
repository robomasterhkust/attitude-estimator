#ifndef _MATH_MISC_H_
#define _MATH_MISC_H_
#include "ch.h"
#include <math.h>

typedef struct {
  uint8_t init;
  float a1;
  float a2;
  float b0;
  float b1;
  float b2;
} lpfilterCoefStruct;

typedef struct {
  lpfilterCoefStruct *coef;
  float data[2];
} lpfilterStruct;

static inline float vector_norm(const float v[], const uint8_t length)
{
  uint8_t i;
  float norm = 0.0f;
  for (i = 0; i < length; i++)
    norm += v[i]*v[i];
  return sqrt(norm);
}

static inline void vector_normalize(float v[], const uint8_t length)
{
  uint8_t i;
  float norm = vector_norm(v, length);
  for (i = 0; i < length; i++)
    v[i] /= norm;
}

static inline void vector3_cross(const float a[3], const float b[3],
  float result[3])
{
  result[0] = a[1]*b[2] - a[2]*b[1];
  result[1] = a[2]*b[0] - a[0]*b[2];
  result[2] = a[0]*b[1] - a[1]*b[0];
}

static inline void q_derivative(const float q[4], const float v[3],
  float dq[4])
{
  dq[0] = 0.5 * (v[0] * -q[1] + v[1] * -q[2] + v[2] * -q[3]);
  dq[1] = 0.5 * (v[0] *  q[0] + v[1] * -q[3] + v[2] *  q[2]);
  dq[2] = 0.5 * (v[0] *  q[3] + v[1] *  q[0] + v[2] * -q[1]);
  dq[3] = 0.5 * (v[0] * -q[2] + v[1] *  q[1] + v[2] *  q[0]);
}

void lpfilter_init(lpfilterStruct* const lp,
  const float sample_freq, const float cutoff_freq);

float lpfilter_apply(lpfilterStruct* const lp, const float input);

#endif
