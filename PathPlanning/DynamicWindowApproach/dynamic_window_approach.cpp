/*

Mobile robot motion planning sample with Dynamic Window Approach

*/

#include <iostream>
#include <math.h>
#include <vector>
#include <tuple>

class Config
{
public:
    double max_speed; // [m/s]
    double min_speed; //[m/s]
    double max_yawrate; // [rad/s]
    double max_accel; // [m/ss]
    double max_dyawrate; // [rad/s]
    double v_reso; // [m/s]
    double yawrate_reso; // [rad/s]
    double dt; // [s]
    double predict_time; // [s]
    double to_goal_cost_gain;
    double speed_cost_gain;
    double robot_radius; // [m]

    Config();
};

Config::Config()
{
    max_speed = 1.0; // [m/s]
    min_speed = -0.5; //[m/s]
    max_yawrate = 40.0 * M_PI / 180.0; // [rad/s]
    max_accel = 0.2; // [m/ss]
    max_dyawrate = 40.0 * M_PI / 180.0; // [rad/s]
    v_reso = 0.01; // [m/s]
    yawrate_reso = 0.1 * M_PI / 180.0; // [rad/s]
    dt = 0.1; // [s]
    predict_time = 3.0; // [s]
    to_goal_cost_gain = 1.0;
    speed_cost_gain = 1.0;
    robot_radius = 1.0; // [m]
}

struct RobotState
{
    double x; // [m]
    double y; // [m]
    double yaw; // [rad]
    double v; // [m/s]
    double omega; // [rad/s]
};

struct Coordinates
{
    double x; // [m]
    double y; // [m]
};

std::vector<double> calc_dynamic_window(RobotState x, Config config)
{
    // Dynamic window from robot specification
    std::vector<double> Vs = {
        config.min_speed, config.max_speed,
        -config.max_yawrate, config.max_yawrate
    };

    // Dynamic window from motion model
    std::vector<double> Vd = {
        x.v - config.max_accel * config.dt,
        x.v + config.max_accel * config.dt,
        x.omega - config.max_dyawrate * config.dt,
        x.omega + config.max_dyawrate * config.dt 
    };

    // {vmin, vmax, yawrate min, yawrate max}
    std::vector<double> dw = {
        std::max(Vs[0], Vd[0]), std::min(Vs[1], Vd[1]),
        std::max(Vs[2], Vd[2]), std::min(Vs[3], Vd[3])
    };
    return dw;
}

// Dynamic Window control
std::tuple<Coordinates, RobotState> dwa_control(RobotState x, Coordinates u, Config config, Coordinates goal, std::vector<Coordinates> ob)
{
    RobotState ltraj = {0.0, 0.0, 0.0, 0.0, 0.0};

    std::vector<double> dw = calc_dynamic_window(x, config);

    std::tie(u, traj = )

    return std::forward_as_tuple(u, traj);
}

int main()
{
    std::cout<<"start!"<<std::endl;

    // initial state {x(m), y(m), yaw(rad), v(m/s), omega(rad/s)}
    RobotState x = {0.0, 0.0, M_PI / 8.0, 0.0 ,0.0};

    // goal position {x(m), y(m)}
    Coordinates goal = {10.0, 10.0};

    // obstacle {{x(m), y(m)}, ...}
    std::vector<Coordinates> ob = {
        {-1.0, -1.0},
        {0.0, 0.0},
        {4.0, 2.0},
        {5.0, 4.0},
        {5.0, 5.0},
        {5.0, 6.0},
        {5.0, 9.0},
        {8.0, 9.0},
        {7.0, 9.0},
        {12.0, 12.0}
    };

    Coordinates u = {0.0, 0.0};
    Config config;
    RobotState traj = x;

    for(int i=0; i<10; i++){ // あとで1000に直す
        RobotState ltraj;
        std::tie(u, ltraj) = dwa_control(x, u, config, goal, ob);

    }

    
    return 0;
}