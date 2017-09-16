#include "math_misc.h"
//write all non-inline function here

void lpfilter_init(lpfilterStruct* const lp,
  const float sample_freq, const float cutoff_freq)
{
  lp->data[0] = lp->data[1] = 0.0f;

  if(lp->coef->init != 1)
  {
    float fr = sample_freq / cutoff_freq;
    float ohm = tanf(M_PI / fr);
    float c = 1.0f + 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm;
    lp->coef->b0 = ohm * ohm / c;
    lp->coef->b1 = 2.0f * lp->coef->b0;
    lp->coef->b2 = lp->coef->b0;
    lp->coef->a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    lp->coef->a2 = (1.0f - 2.0f * cosf(M_PI / 4.0f) * ohm + ohm * ohm) / c;

    lp->coef->init = 1;
  }
}

float lpfilter_apply(lpfilterStruct* const lp, const float input)
{
  float delay = input - lp->data[0] * lp->coef->a1 - lp->data[1] * lp->coef->a2;

  // don't allow bad values to propagate via the filter
  if(!isfinite(delay))
    delay = input;

  float output = delay * lp->coef->b0 +
              lp->data[0] * lp->coef->b1 +
              lp->data[1] * lp->coef->b2;

  lp->data[1] = lp->data[0];
  lp->data[0] = delay;

  // return the value.  Should be no need to check limits
  return output;
}
