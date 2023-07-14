#pragma once

#include "veins/modules/application/cbtcSim/base80211p/CBTCBaseApplLayer.h"
#include "veins/modules/application/cbtcSim/sim1Modules/sim1Parameters.h"

#include <iostream>
#include <fstream>


namespace veins {

static SIM1Parameters params;

class VEINS_API Train11p : public CBTCBaseApplLayer {

public:
    void initialize(int stage) override;
    void finish() override;


private:
    std::string trainID;
    WaysideOperationalMessage newestWOM;

private:
    void setNewestWOM(WaysideOperationalMessage wom);


protected:
    void onWOM(WaysideOperationalMessage* wom) override;
    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;


protected:

    std::string op_phase; //train's current operational phase
    double max_spd_stp_dist; //the distance covered while stopping a train operating with maximum speed using maximum service braking deceleration

};

} // namespace veins
