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

struct Speeds
{
    double v; // [m/s]
    double omega; // [rad]
};

RobotState motion(RobotState x, Speeds u, double dt);
std::vector<double> calc_dynamic_window(RobotState x, Config config);
std::vector<RobotState> calc_trajectory(RobotState xinit, double v, double y, Config config);
std::tuple<Speeds, std::vector<RobotState>> calc_final_input(RobotState x, Speeds u, std::vector<double> dw, Config config, Coordinates goal, std::vector<Coordinates> ob);
double calc_obstacle_cost(std::vector<RobotState> traj, std::vector<Coordinates> ob, Config config);
double calc_to_goal_cost(std::vector<RobotState> traj, Coordinates goal, Config config);
std::tuple<Speeds, std::vector<RobotState>> dwa_control(RobotState x, Speeds u, Config config, Coordinates goal, std::vector<Coordinates> ob);

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
        {0.0, 2.0},
        {4.0, 2.0},
        {5.0, 4.0},
        {5.0, 5.0},
        {5.0, 6.0},
        {5.0, 9.0},
        {8.0, 9.0},
        {7.0, 9.0},
        {12.0, 12.0}
    };

    Speeds u = {0.0, 0.0};
    Config config;
    std::vector<RobotState> traj = {x};

    for(int i=0; i<1000; i++){ // あとで1000に直す
        std::vector<RobotState> ltraj;
        std::tie(u, ltraj) = dwa_control(x, u, config, goal, ob);

        x = motion(x, u, config.dt);

        traj.emplace_back(x);

        // check goal
        if(sqrt(pow(x.x - goal.x , 2) + pow(x.y - goal.y , 2)) <= config.robot_radius){
            std::cout << "Goal!" << std::endl;
            break;
        }

    }

    std::cout << "Done" << std::endl;

    
    return 0;
}


RobotState motion(RobotState x, Speeds u, double dt)
{   // motion model 

    x.yaw += u.omega * dt;
    x.x += u.v * cos(x.yaw) * dt;
    x.y += u.v * sin(x.yaw) * dt;
    x.v = u.v;
    x.omega = u.omega;

    return x;
}

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

std::vector<RobotState> calc_trajectory(RobotState xinit, double v, double y, Config config)
{
    RobotState x = xinit;
    std::vector<RobotState> traj = {x};
    double time = 0.0;

    while(time <= config.predict_time){

        Speeds u = {v, y};
        x = motion(x, u, config.dt);
        traj.emplace_back(x);
        time += config.dt;
    }

    return traj;
}

std::tuple<Speeds, std::vector<RobotState>> calc_final_input(RobotState x, Speeds u, std::vector<double> dw,
                                                     Config config, Coordinates goal, std::vector<Coordinates> ob)
{
    RobotState xinit = x;
    double min_cost = 10000.0;
    Speeds min_u = u;
    min_u.v = 0.0;
    std::vector<RobotState> best_traj = {x};

    // evalucate all trajectory with sampled input in dynamic window
    for(double v = dw[0]; v < dw[1]; v += config.v_reso){
        for(double y = dw[2]; y < dw[3]; y += config.yawrate_reso){
            std::vector<RobotState> traj =calc_trajectory(xinit, v, y, config);

            // calc cost
            double to_goal_cost = calc_to_goal_cost(traj, goal, config);
            double speed_cost = config.speed_cost_gain * (config.max_speed - traj.back().v);
            double ob_cost = calc_obstacle_cost(traj, ob, config);

            double final_cost = to_goal_cost + speed_cost + ob_cost;

            // search minimum trajectory
            if(min_cost >= final_cost){
                min_cost = final_cost;
                min_u.v = v;
                min_u.omega = y;
                best_traj = traj;
            }
        }
    } 
    
    return std::forward_as_tuple(min_u, best_traj);
}

double calc_obstacle_cost(std::vector<RobotState> traj, std::vector<Coordinates> ob, Config config)
{   // calc obstacle cost inf: collistion, 0:free
    
    const int skip_n = 2;
    double minr = INFINITY;

    for(int ii=0; ii<traj.size(); ii += skip_n){
        for(int i=0; i<ob.size(); i++){
            double ox = ob[i].x;
            double oy = ob[i].y;
            double dx = traj[ii].x - ox;
            double dy = traj[ii].y - oy;

            double r = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
            if( r <= config.robot_radius){
                return INFINITY;
            }
            if( minr >= r){
                minr = r;
            }
        }
    }

    return 1.0 / minr;
}

double calc_to_goal_cost(std::vector<RobotState> traj, Coordinates goal, Config config)
{   // calc to goal cost. It is 2D norm.

    double goal_magnitude = sqrt(pow(goal.x, 2.0) + pow(goal.y, 2.0) );
    double traj_magnitude = sqrt(pow(traj.back().x, 2.0) + pow(traj.back().y, 2.0) );
    double dot_product = (goal.x * traj.back().x) + (goal.y * traj.back().y);
    double error = dot_product / (goal_magnitude * traj_magnitude);
    double error_angle = acos(error);
    double cost = config.to_goal_cost_gain * error_angle;

    return cost;
}

// Dynamic Window control
std::tuple<Speeds, std::vector<RobotState>> dwa_control(RobotState x, Speeds u, Config config, Coordinates goal, std::vector<Coordinates> ob)
{
    RobotState ltraj = {0.0, 0.0, 0.0, 0.0, 0.0};

    std::vector<double> dw = calc_dynamic_window(x, config);

    std::vector<RobotState> traj = {};
    std::tie(u, traj) = calc_final_input(x, u, dw, config, goal, ob);

    return std::forward_as_tuple(u, traj);
}
