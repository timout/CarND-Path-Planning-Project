#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <string>

#include <memory>

using namespace std;

const double MAX_SPEED = (50.0 / 2.24) - 0.3;
const double MAX_DISTANCE = 10000;
const double OPEN_LANE = 100;
const double BACK_OPEN_LANE = 50;
const double TOO_CLOSE = 15;
const double CHECK_CHANGE = 20;

const double REDUCE_SPEED_DISTANCE = 10;
const double KEEP_LANE_DISTANCE = 30;

const int LANE_NUM = 3;

const double ACCELERATION_LIMIT = 0.16; 

struct Lane
{
    int id;
    double head_distance;
    double head_velocity;
    double back_distance;
    double back_velocity;

    Lane(int id) : 
        id(id), head_distance(MAX_DISTANCE), head_velocity(MAX_SPEED), back_distance(MAX_DISTANCE), back_velocity(MAX_SPEED) {}
};

struct State 
{
    const double current_s;
    const double current_d;
    const int current_lane;
    const vector<unique_ptr<Lane>> lanes;
    double velocity = 0;
    double last_velocity = 0;

    vector<double> scores = {0,0,0};

    State(double s, double d, int l);
    ~State() {}
};

class Planner 
{
private:
    unique_ptr<State> state = unique_ptr<State>(new State(0,0,0));
public:
    Planner() {}

    ~Planner() {}

    int execute(vector<double> & frenet_coordinates, vector<vector<double>> sensor_fusion, double current_car_s);

    double get_next_velocity(); 

private:

    void init_lanes(const vector<vector<double>> & sensor_fusion);
    void init_lane(const vector<double> & vehicle);

    vector <double> evaluate_lanes();
    double evaluate_lane(int lane);

    int select_lane(vector <double> & scores);
    int validate(int lane, vector <double> & scores, double current_d);

    void create_state(double s, double d, int l);

};

#endif