#ifndef __SIM_1_PARAMETERS_H_
#define __SIM_1_PARAMETERS_H_

#pragma once
#include <vector>
#include <string>


class SIM1Parameters {

public:

//Trips
    int tripID = 1; //can be used to identify a trip when generating a dataset with multiple trips

//Train Movement Model:
    double Xdep = 2500; //point of departure
    double Vcru = 20; //average velocity during cruising phase
    double Tcru; //time to start cruising
    double Sdec; //distance covered during deceleration phase; or braking distance
    double safetyMargin = 25; //the minimum separation distance between trains

    double decel; //service braking
    double emergency_decel; //emergency braking


//Stations:
    std::vector<double> stationsLocs{2500, 5000, 7500, 10000, 12500, 15000, 17500, 20000, 22500}; //locations of stations on the track
    std::string stopsRoadId = "E";
    uint8_t stopsLaneId = 0;
    int dwellTime = 30; //waiting time at each station
    int emergencyStopTime = 60; //emergency stopping time


//thresholds for overspeed protection
    double acceptable_spd_limit = 7.5; //boundary of the percentage of acceptable deviation
    double overspeed_limit = 15; //boundary of the percentage of adjustable deviation, before applying emergency braking


//train operational phases
    std::string ACCELERATING_PHASE = "ACCELERATING";
    std::string CRUISING_PHASE = "CRUISING";
    std::string DECELERATING_PHASE = "DECELERATING";
    std::string STOPPING_PHASE = "STOPPING";

    std::string EMERGENCY_BRAKING_PHASE = "EMERGENCY_BRAKING";
    std::string EMERGENCY_STOPPING_PHASE = "EMERGENCY_STOPPING";


//generating datasets
    bool generateDatasets = true; //if set to true, it generates two datasets from the received TSMs and WOMs by WSUSs and trains


};

#endif
