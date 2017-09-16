/**
 * 20170514 Beck Pang
 * @brief timercmp functions reimplemention
 */
#ifndef _TIMERCMP_H_
#define _TIMERCMP_H_

#include <sys/time.h>

void timeradd(struct timeval *a, struct timeval *b, struct timeval *result);

void timersub(struct timeval *a, struct timeval *b, struct timeval *result);

void timerclear(struct timeval *tvp);

int timerisset(struct timeval *tvp);

int timercmp(struct timeval *a, struct timeval *b);

float timerfloat(struct timeval *a);

unsigned long timerlong(struct timeval *a);

#endif
