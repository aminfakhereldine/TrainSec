#pragma once

#include "veins/modules/application/cbtcSim/base80211p/CBTCBaseApplLayer.h"
#include "veins/modules/application/cbtcSim/sim1Modules/sim1Parameters.h"

namespace veins {

static SIM1Parameters params;
enum speedStatus{downspeeding, acceptable, overspeeding, dangerous};

class VEINS_API WSU11p : public CBTCBaseApplLayer {

public:
    void initialize(int stage) override;
    void finish() override;


private:
    std::string wsuID;
    int index; //WSU index in the list of WSUs

    //the following vectors store TSMs received from trains in the periods between sending wayside updates
    std::vector<TrainStatusMessage> tsmsBeforeWSU; //stores TSMs received from trains at positions before the WSU's station
    std::vector<TrainStatusMessage> tsmsAfterWSU; //stores TSMs received from trains at positions after the WSU's station

private:
    void storeTSM(TrainStatusMessage tsm);
    void sortTSMs(std::vector<TrainStatusMessage>& tsms);
    double distToPrecTrain(TrainStatusMessage& tsm_T1, TrainStatusMessage& tsm_T2);
    bool areSafelySeparated(TrainStatusMessage& tsm_T1, TrainStatusMessage& tsm_T2);
    speedStatus checkSpeedStatus(TrainStatusMessage& tsm);

protected:
    void onTSM(TrainStatusMessage* tsm) override;
    void handleSelfMsg(cMessage* msg) override;


protected:

};

} // namespace veins



