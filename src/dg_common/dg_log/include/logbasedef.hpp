#ifndef BASE_LOG_LOG_HPP
#define BASE_LOG_LOG_HPP

#include <stdarg.h>
#ifndef _WIN32
#define MINCHAR     0x80
#define MAXCHAR     0x7f
#define MINSHORT    0x8000
#define MAXSHORT    0x7fff
#define MINLONG     0x80000000
#define MAXLONG     0x7fffffff
#define MAXBYTE     0xff
#define MAXWORD     0xffff
#define MAXDWORD    0xffffffff
#endif
#pragma once
struct SystemTime {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int milli_second;
};
inline SystemTime get_local_time(time_t seconds)
{
    struct tm tm_now;
    localtime_r(&seconds, &tm_now);
    return SystemTime{
        tm_now.tm_year + 1900,
        tm_now.tm_mon + 1,
        tm_now.tm_mday,
        tm_now.tm_hour,
        tm_now.tm_min,
        tm_now.tm_sec,
        0
    };
}
inline void sec2str(char * str,time_t seconds)
{
    struct tm tm_now;
    tm_now = *localtime(&seconds);
    //localtime_r(&seconds, &tm_now);
    sprintf(str,"%d-%d-%d %d:%d:%d",tm_now.tm_year + 1900,tm_now.tm_mon + 1,
            tm_now.tm_mday, tm_now.tm_hour,tm_now.tm_min,tm_now.tm_sec );
}
inline SystemTime get_local_time1()
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    struct tm tm_now;
    localtime_r(&tv_now.tv_sec, &tm_now);

    return SystemTime{
        tm_now.tm_year + 1900,
        tm_now.tm_mon + 1,
        tm_now.tm_mday,
        tm_now.tm_hour,
        tm_now.tm_min,
        tm_now.tm_sec,
        (int)tv_now.tv_usec / 1000
    };
}

#endif  // BASE_LOG_LOG_HPP

