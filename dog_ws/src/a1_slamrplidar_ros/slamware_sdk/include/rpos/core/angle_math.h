/*
* angle_math.h
* Angle math utility functions
*
* This is moved from the anglehelper.h
* This should be in onlineslam project, but be placed here for depended by the aperture class
*
* Created by Tony Huang (tony@slamtec.com) at 2017-1-10
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once
#include <limits>
#include <cstdint>
#include <cmath>
#include <float.h>

// include this header to include the M_PI constant
#include <Eigen/Geometry>

namespace rpos{ namespace core{
    
    // constrain the value to [begin, end) range
    template<typename T>
    static inline T constrain(T begin, T end, T v)
    {
        if (begin > end) {
            return constrain(end, begin, v);
        }
        if (v < end && v >= begin) {
            return v;
        }
        const T size = end - begin;// -(std::numeric_limits<T>::epsilon)();
        if (size < (std::numeric_limits<T>::epsilon)()) {
            return v;
        }
        T result = (T)(begin + (T)std::fmod((T)((T)(std::fmod(v - begin, size)) + size), size));
#if 1
        if (result >= end || result < begin) {
            std::printf("##################################################################################################\n");
            std::printf("begin: %16.14f %d end: %16.14f %d \n", begin, *((std::int32_t*)(&begin)), end, *((std::int32_t*)(&end)));
            std::printf("result: %16.14f %d intput: %16.14f %d \n", result, *((std::int32_t*)(&result)), v, *((std::int32_t*)(&v)));
            std::printf("##################################################################################################\n");
        }
#endif
        return result;
    }

    static inline float constraitRadZeroTo2Pi(float v)
    {
        return constrain(0.f, float(2 * M_PI), v);
    }

    static inline float constraitRadNegativePiToPi(float v)
    {
        return constrain(float(-M_PI), float(M_PI), v);
    }

    static inline double constraitRadZeroTo2Pi(double v)
    {
        return constrain(0., 2 * M_PI, v);
    }

    static inline double constraitRadNegativePiToPi(double v)
    {
        return constrain(-M_PI, M_PI, v);
    }

    static inline float angleDistanceZeroTo2Pi(float normalizedDest, float normalizedSrc)
    {
        return constraitRadZeroTo2Pi(normalizedDest - normalizedSrc);
    }
    static inline double angleDistanceZeroTo2Pi(double normalizedDest, double normalizedSrc)
    {
        return constraitRadZeroTo2Pi(normalizedDest - normalizedSrc);
    }

    static inline float angleDistanceNegativePiToPi(float dest, float src)
    {
        return constraitRadNegativePiToPi(dest - src);
    }
    static inline double angleDistanceNegativePiToPi(double dest, double src)
    {
        return constraitRadNegativePiToPi(dest - src);
    }

    // compare two angle in the context of 0-2PI 
    // returns 0:  a1 and a2 are coincide
    // returns 1:  a1 is ahead of a2 in the CW rotation direction
    // returns -1: a1 is behind of a2
    static inline int angleCompare(float norm_a1, float norm_a2)
    {
        float delta = angleDistanceNegativePiToPi(norm_a1, norm_a2);
        if (fabs(delta) < 0.00001f) return 0;

        if (delta < 0) {
            return -1;
        }
        else {
            return 1;
        }
    }

    static inline double rad2deg(double rad)
    {
        return rad * 180 / M_PI;
    }

    static inline double deg2rad(double deg)
    {
        return deg * M_PI / 180;
    }

    static inline float rad2deg(float rad)
    {
        return float(rad * 180 / M_PI);
    }

    static inline float deg2rad(float deg)
    {
        return float(deg * M_PI / 180);
    }

} }

