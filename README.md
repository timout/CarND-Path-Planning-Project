# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Implementation

Implementation consist of 3 parts:
 - Preparation for path planning
 - Path planning
 - Trajectory generation

#### Preparation 
That part use the following steps:
- Initialize with simulator's data (car's: x,y,yaw)
- If there is a remaning path:   
  It reuses it to send it back to simulator.  
  Calculates parameters (x,y, yaw) using last 2 items from previous path.  
- Converts x,y coordinates to frenet coordinates (s,d) using `getFrenet`  helper function
- Feed frenet coordinates and sensors data from simulator to path planner.

#### Path Planner
Planner accepts sensors data, `s` and `d` coordinates from end of the path and using the following steps produces next path point `d` coordinate for the car.  
###### Steps
- Process sensors data for each lane to get parameters of closest vehicles (distance and velocity).
- Calculate scores for each lane using processed sensor data and car path coordinates.
- Using calculated scores decide if current lane needs to be changed.
- If lane is changed - validates if it actually can be changed.
- Return result lane d coordinate.

#### Trajectory generator

It uses third party cubic `spline` library to build smooth trajectory for the car.
###### Steps
- To build smooth path - generate 3 waypoints.  
  I have tryed various spacing for waypoints starting from 30 m and end up with using 45 m. Shorter spacing can cause to exceding max jerk. 
- Using waypoints and `spline` library - build path points for the car.  
  Distance between points is calculated by using point velocity and 0.02 second as time.  
  Velocity for each point is calculated using target velocity provided by planner so it can be gradually increased or decreased. 
- Convert coordinates to make it car' local
- Push calculated path to simulator
Every point 
- Using old path points and waypoint `spline` 

#### Things to improve
1. This path planner is very simple, it uses very simple logic to plan the path. 
2. Planer is one way planner without ability to change already made decision.
3. Trajectory generation can be made more advaced: time to time it cause exceding jerk or car goes out of lane.
