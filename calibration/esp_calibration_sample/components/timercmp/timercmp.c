/**
 * 20170514, Beck Pang
 * @brief time compare functions reimplementing
 * @source https://linux.die.net/man/3/timercmp
 * originally defined in esp-idf/components/newlib/include/sys/time.h
 */
#include "timercmp.h"
void timeradd(struct timeval *a, struct timeval *b, struct timeval *result)
{
    result->tv_sec  = a->tv_sec + b->tv_sec;
    result->tv_usec = a->tv_usec + b->tv_usec;
    if (result->tv_usec >= 1000000)
    {
        ++result->tv_sec;
        result->tv_usec -= 1000000;
    }
}


void timersub(struct timeval *a, struct timeval *b, struct timeval *result)
{
    result->tv_sec  = a->tv_sec - b->tv_sec;
    result->tv_usec = a->tv_usec - b->tv_usec;
    if (result->tv_usec < 0)
    {
        --result->tv_sec;
        result->tv_usec += 1000000;
    }
}


void timerclear(struct timeval *tvp)
{
    tvp->tv_sec  = 0;
    tvp->tv_usec = 0;
}


int timerisset(struct timeval *tvp)
{
    return(tvp->tv_sec || tvp->tv_usec);
}


int timercmp(struct timeval *a, struct timeval *b)
{
    if (a->tv_sec == b->tv_sec)
    {
        return a->tv_usec - b->tv_usec;
    }
    else
    {
        return a->tv_sec - b->tv_sec;
    }
}

float timerfloat(struct timeval *a)
{
	return (float)a->tv_sec + (float)a->tv_usec / 1000000.0f;
}

unsigned long timerlong(struct timeval *a)
{
	return a->tv_sec * 1000000.0f + a->tv_usec;
}
