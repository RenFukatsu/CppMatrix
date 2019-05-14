#ifndef MOTION_MODEL_HPP
#define MOTION_MODEL_HPP

#include <iostream>
#include <math.h>
#include <tuple>

// motion parameter
double L = 1.0; // wheel base
double ds = 0.1; // course distance
double v = 10.0 / 3.6; // velocity [m/s]

class State
{
public:
    double x;
    double y;
    double yaw;
    double v;

    State();
    State(double, double, double, double);
};


double pi_2_pi(double angle);
State update(State state, double v, double delta, double dt, double L);
std::tuple<double, double, double> genetate_tragectory();
std::tuple<double, double, double> generate_last_state();


inline State::State()
{
    x = 0.0;
    y = 0.0;
    yaw = 0.0;
    v = 0.0;
}

inline State::State(double x, double y, double yaw, double v)
{
    x = x;
    y = y;
    yaw = yaw;
    v = v;
}

inline double pi_2_pi(double angle)
{
    return fmod(angle + M_PI, 2 * M_PI) - M_PI;
}

inline State update(State state, double v, double delta, double dt, double L)
{
    state.v = v;
    state.x = state.x + state.v * cos(state.yaw) * dt;
    state.y = state.y + state.v * sin(state.yaw) * dt;
    state.yaw = state.yaw + state.v / L * tan(delta) * dt;
    state.yaw = pi_2_pi(state.yaw);

    return state;
}

inline std::tuple<double, double, double> genetate_tragectory()
{
    
    return std::forward_as_tuple(x, y, yaw);
}

inline std::tuple<double, double, double> generate_last_state()
{
    return std::forward_as_tuple(state.x, state.y, state.yaw);
}

#endif // MODEL_MOTION_HPP