/*

Model trajectory generator

*/

#ifndef MODEL_PREDICTIVE_TRAJECTORY_GENERATOR_HPP
#define MODEL_PREDICTIVE_TRAJECTORY_GENERATOR_HPP

#include <iostream>
#include <vector>
#include <math.h>
#include "motion_model.hpp"

// optimization parameter
int max_iter = 100;
std::vector<double> h = {0.5, 0.02, 0.02};
double cost_th = 0.1;

bool show_animation = true;

std::vector<double, double, double> calc_diff(State target, double x, double y, double yaw)
{
    std::vector<double, double, double> d = {target.x - x, target.y - y, pi_2_pi(target.yaw - yaw)};

    return d;
}



#endif // MODEL_PREDICTIVE_TRAJECTORY_GENERATOR_HPP