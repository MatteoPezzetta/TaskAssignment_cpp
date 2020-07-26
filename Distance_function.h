#ifndef DISTANCE_FUNCTION_H
#define DISTANCE_FUNCTION_H

// Declaration of function l1_norm() to compute the Manhattan distance
double l1_norm(std::array<double, 2>& size2_array);

// Declaration of function tt() to compute the travel time betwwen two cartesian coordinates
double tt(std::array<double, 2>& orgn, std::array<double, 2>& dest);

#endif