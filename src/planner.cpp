

#include <math.h>

#include <iostream>

#include "planner.hpp"

//d to lane
int get_lane(int d) {
  return d >> 2;
}

// lane to d
int lane_to_d(int lane) {
  return (lane << 2) + 2;
}

vector<unique_ptr<Lane>> init_lanes(int size)
{
    vector<unique_ptr<Lane>> lanes;
    for (int i = 0; i < size; ++i) lanes.push_back(unique_ptr<Lane>(new Lane(i)));
    return lanes;
}

State::State(double s, double d, int l) : 
    current_s(s), current_d(d), current_lane(l), lanes(init_lanes(LANE_NUM))
{
}

void Planner::create_state(double s, double d, int l)
{
    double lv = this->state->last_velocity;
    this->state = unique_ptr<State>(new State(s, d, l));
    this->state->last_velocity = lv;
}

int Planner::execute(vector<double> & frenet_coordinates, vector<vector<double>> sensor_fusion, double current_car_s) 
{
    double s = frenet_coordinates[0];
    double d = frenet_coordinates[1];
    int l = get_lane(d); //current lane
    create_state(s, d, l); // create initial path state
    init_lanes(sensor_fusion); // process sensors data 
    vector <double> scores = evaluate_lanes(); // calculates scores
    int next_lane = select_lane(scores);  // select path lane
    next_lane = validate(next_lane, scores, current_car_s); // validate
    // if plenty space a head of the car stay on current lane and set target velocity as max
    if ( next_lane == state->current_lane && state->lanes[next_lane]->head_distance > REDUCE_SPEED_DISTANCE ) {
        state->velocity = MAX_SPEED;
    } else {
        // set target velocity = next car ahead.
        state->velocity = state->lanes[next_lane]->head_velocity;
    }
    //update scores for calculating average . TODO: 
    for ( int l = 0; l < LANE_NUM; ++l ) state->scores[l] = scores[l];
    return lane_to_d(next_lane);
}
/*
Process sensors data by calculating distance and velocity from vehicles around the car 
*/
void Planner::init_lanes(const vector<vector<double>> & sensor_fusion) {
    for ( const vector<double> & vehicle : sensor_fusion ) {
        double d = vehicle[6];
        int l = get_lane(d);
        if ( l >= 0 && l < LANE_NUM ) {
            init_lane(vehicle);
        }
    }
}

/*
Process vehicle's sensor data to find if it is closest vehicle to the cat on vehicle's lane
*/
void Planner::init_lane(const vector<double> & vehicle) {
    double d = vehicle[6];
    int l = get_lane(d);
    double x = vehicle[3];
    double y = vehicle[4];
    double s = vehicle[5];
    double v = sqrt( x * x + y * y );
    if ( s >= state->current_s ) { // vehicle is ahead of the car
        double dist = s - state->current_s;
        if ( state->lanes[l]->head_distance > dist ) {
            state->lanes[l]->head_distance = dist;
            state->lanes[l]->head_velocity = v;
        }
    } else if ( s < state->current_s ) { //vehicle is behind the car
        double dist = state->current_s - s;
        if ( state->lanes[l]->back_distance > dist ) {
            state->lanes[l]->back_distance = dist;
            state->lanes[l]->back_velocity = v;
        }
    }
}

/** 
 * Calculate scores for each lane
 */
vector <double> Planner::evaluate_lanes() {
  vector <double> scores(LANE_NUM, 0.0);
  //if plenty of space ahead of the car - skip calculation and keep current lane
  if ( state->lanes[state->current_lane]->head_distance  > KEEP_LANE_DISTANCE ) {
      scores[state->current_lane] = 10;
      return scores;
  }  
  for ( int l = 0; l < LANE_NUM; ++l ) {
      double score = evaluate_lane(l);
      scores[l] = score;
  }
  return scores;
}

/**
 * Clculate score for lane
 */ 
double Planner::evaluate_lane(int lane) 
{
    double score = ( lane == state->current_lane ) ? 0.3 : 0; // add priority to current lane
    double hd = state->lanes[lane]->head_distance;
    double bd = state->lanes[lane]->back_distance;
    double hv = state->lanes[lane]->head_velocity;
    double bv = state->lanes[lane]->back_velocity;

    if ( hd > OPEN_LANE && bd > BACK_OPEN_LANE) {
        return score + 10;  // empty line - highest score
    }
    if ( hd < TOO_CLOSE || bd < TOO_CLOSE) {
        return score - 10; // line with vihicles at too-close distance - lowest score
    }
    score += hd * 0.6 / 40.0; // open space ahead gives more scores
    score += bd * 0.4 / 40.0; // open space behind gives more scores
    score += (hv / MAX_SPEED ) * 0.3; // faster closest head vehicle gives more scores
    score += (bv * 0.2 / MAX_SPEED); // slower closest behind vehicle gives more scores
    return score;
}

/**
 * Select lane
 */
int Planner::select_lane(vector <double> & scores)
{ //TODO: make LANE_NUM dependent
    int l = state->current_lane;
    if (l == 0) { // if left lane - compare left and middle only
        return ( scores[1] > scores[0] ) ? 1 : 0;
    } else if (l == 1) { //if middle choose from all 3 lanes
        if ( scores[2] > scores[0] && scores[2] > scores[1] ) return 2;
        if ( scores[0] > scores[1] && scores[0] > scores[2] ) return 0;
        return 1;
    } else { // if right - compare right and middle onle
        return ( scores[2] > scores[1] ) ? 2 : 1;
    }
}

/**
 * Get next velocity for the point on the path
 * To insure smooth acceleration  
 */
double Planner::get_next_velocity() 
{
    double v = state->velocity;
    double lv = state->last_velocity;
    if ( lv < v - ACCELERATION_LIMIT ) {
       lv = lv + ACCELERATION_LIMIT;
    } else if ( lv > v + ACCELERATION_LIMIT ) {
        lv = lv - ACCELERATION_LIMIT;
    }
    state->last_velocity = lv;
    return lv;
}

/** 
 * Validate next selected lane
 */
int Planner::validate(int lane, vector <double> & scores, double current_s)
{
    if ( lane != state->current_lane ) {
        //cout << "Try Lane change: " << state->current_lane << " to " << lane << endl;
        if ( state->lanes[lane]->head_distance < TOO_CLOSE || state->lanes[lane]->back_distance < TOO_CLOSE) {
            return state->current_lane;
        }
        if ( state->lanes[lane]->head_distance < CHECK_CHANGE && state->lanes[lane]->head_velocity < state->last_velocity ) {
            return state->current_lane;
        }
        if ( state->lanes[lane]->back_distance < CHECK_CHANGE && state->lanes[lane]->back_velocity > state->last_velocity ) {
            return state->current_lane;
        }
        double d = state->current_s - current_s;
        if ( abs(d - state->lanes[lane]->back_distance ) < TOO_CLOSE ) {
            //cout << "cur s to close" << endl;
            return state->current_lane;
        }
        //cout << "Lane changed: " << state->current_lane << " to " << lane << endl;
    }
    return lane;
}

