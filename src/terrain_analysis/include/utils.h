/**
  ******************************************************************************
  * @file    utils.h
  * @author  Alex Liu 
  * @version V1.0.0
  * @date    2021/12/27
  * @brief   utils for MobiRo @ tib_k331
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#ifndef UTILS_H
#define UTILS_H

#include <cmath>

namespace tib_k331_perception {
//
template <typename T>
inline T square(T x) {return x * x;}

template <typename T>
inline T myabs(T x) {return (x >= 0)?x:-x;}

template <typename T>
inline void UpperBound(T &x, T bound) {
  x = (x > bound)?bound:x;
}

template <typename T>
inline void LowerBound(T &x, T bound) {
  x = (x < bound)?bound:x;
}

template <typename T>
inline T EuclideanNorm(T dx, T dy) {
  return std::sqrt(square(dx) + square(dy));
}

template <typename T>   
inline T EuclideanNorm(const T *pt_a, const T *pt_b) {
  return EuclideanNorm(pt_a[0] - pt_b[0], pt_a[1] - pt_b[1]);
}

inline void PhaseCorrection(double &angle) {
  if (angle > M_PI) angle -= 2 * M_PI;
  if (angle < -M_PI) angle += 2 * M_PI;
}
//
}  // namespace tib_k331_perception

#endif  // UTILS_H