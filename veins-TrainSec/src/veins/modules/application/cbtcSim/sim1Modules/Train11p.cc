#include "veins/modules/application/cbtcSim/sim1Modules/Train11p.h"

#include <iostream>
#include <fstream>

#include "veins/base/phyLayer/PhyToMacControlInfo.h"
#include "veins/modules/phy/DeciderResult80211.h"

using namespace veins;
using namespace std;

Define_Module(veins::Train11p);

//train's initialization
void Train11p::initialize(int stage)
{
    //initializing train
    CBTCBaseApplLayer::initialize(stage);

    if (stage == 0) {


        //create train ID
        trainID = mobility->getExternalId(); //setting the train ID (same ID in SUMO)
        max_spd_stp_dist = pow(traci->vehicle(trainID).getMaxSpeed(), 2) / (2 * traci->vehicle(trainID).getDeccel());

        //computing Tcru: a train will speed up from zero to Vcru using its acceleration
        //equation used: difference_speed = acceleration * difference_time
        params.Tcru = params.Vcru / traci->vehicle(trainID).getAccel();

        //computing Sdec: assuming the train is operating at maximum speed and will slow down to zero using service braking
        //equation used: Vfinal^2 - Vinitial^2 = 2 * acceleration * displacement
        params.Sdec = pow(traci->vehicle(trainID).getMaxSpeed(), 2) / (2 * traci->vehicle(trainID).getDeccel());

        params.decel = traci->vehicle(trainID).getDeccel();
        params.emergency_decel = traci->vehicle(trainID).getEmergencyDecel();


        //start the first acceleration phase:
        op_phase = params.ACCELERATING_PHASE;
        traci->vehicle(trainID).slowDown(params.Vcru, params.Tcru);



        if(params.generateDatasets){
            //add the features' names if the dataset file didn't exist before
            std::string filename = "womFile.csv";
            std::ifstream file(filename);

            if(!file){
                ofstream womFile;
                womFile.open("womFile.csv",ios::app);
                assert(!womFile.fail());
                womFile << "tripID,rcvTime,sendTime,wsuID,wsuPosx,trainID,LMA,gtLMA,applyEB,adjustSpd,desiredSpd,rssi,attackType" << endl;
                womFile.close();
                assert(!womFile.fail());
            }

        }

    }

}


//train's behavior upon receiving a WaysideOperationalMessage (WOM) from a WSU
void Train11p::onWOM(WaysideOperationalMessage* wom)
{

    if(trainID.compare(wom->getReceiverTrainId()) == 0){

        double rssi = check_and_cast<DeciderResult80211*>(check_and_cast<PhyToMacControlInfo*>(wom->getControlInfo())->getDeciderResult())->getRecvPower_dBm();

        if(params.generateDatasets){
            ofstream womFile;
            womFile.open("womFile.csv",ios::app);
            assert(!womFile.fail());

            womFile << params.tripID << "," << simTime().dbl() << "," << wom->getSendTime().dbl() << "," << wom->getSenderWSUId() << "," <<
                    wom->getSenderWSUPosition().x << "," <<  trainID.c_str() << "," << wom->getLMA() << "," << wom->getGtLMA() << "," << wom->getApplyEmergencyBraking() << "," <<
                    wom->getAdjustSpeed() << "," << wom->getDesiredSpeed() << "," << rssi << "," << wom->getAttackType() << endl;

            womFile.close();
            assert(!womFile.fail());
        }


        setNewestWOM(*wom);
    }

}


void Train11p::handleSelfMsg(cMessage* msg)
{
    //populate and send a TSM
    switch (msg->getKind()) {

    case SEND_TSM_EVT: {

        //Sending TSMs periodically
        TrainStatusMessage* tsm = new TrainStatusMessage();
        tsm->setSendTime(simTime());
        tsm->setSenderTrainId(trainID.c_str());
        tsm->setAttackType("NORMAL");

        if(attackType.compare("RANDOM_POSITION_OFFSET") == 0 && randomNumber() < attackProbability) {
            Coord coord = mobility->getPositionAt(simTime());
            double randomOffset = randomNumber() * (upperPositionOffset - lowerPositionOffset) + lowerPositionOffset;
            coord.x += randomOffset;
            tsm->setTrainPosition(coord);
            tsm->setAttackType(attackType.c_str());
        }

        else {
            tsm->setTrainPosition(mobility->getPositionAt(simTime()));
        }


        //position ground truth
        tsm->setGtTrainPosition(mobility->getPositionAt(simTime()));

        if(attackType.compare("RANDOM_SPEED_OFFSET") == 0 && randomNumber() < attackProbability) {
            double randomOffset = randomNumber() * (upperSpeedOffset - lowerSpeedOffset) + lowerSpeedOffset;
            tsm->setTrainSpeed(mobility->getSpeed()+randomOffset);
            tsm->setAttackType(attackType.c_str());
        }

        else {
            tsm->setTrainSpeed(mobility->getSpeed());
        }


        //speed ground truth
        tsm->setGtTrainSpeed(mobility->getSpeed());

        tsm->setTrainAcceleration(traci->vehicle(trainID).getAcceleration());
        tsm->setTrainLength(traci->vehicle(trainID).getLength());
        tsm->setMaxSpdStpDist(max_spd_stp_dist);
        tsm->setOpPhase(op_phase.c_str());


        populateWSM(tsm);


        if(attackType.compare("RANDOM_TSM_DELAY") == 0 && randomNumber() < attackProbability) {
            double randomDelay = randomNumber() * (upperTSMDelay - lowerTSMDelay) + lowerTSMDelay;
            tsm->setAttackType(attackType.c_str());
            sendDelayedDown(tsm, randomDelay);
        }

        else{
            sendDown(tsm);
        }

        scheduleAt(simTime() + beaconInterval, sendTSMEvt);
        break;

    }

    default: {
        CBTCBaseApplLayer::handleSelfMsg(msg);
        break;
    }

    }

}


void Train11p::handlePositionUpdate(cObject* obj)
{

    //Train Movement Model

    if(newestWOM.getApplyEmergencyBraking()){
        //set the operational phase to emergency braking, only if it is not in emergency braking or stopping
        if(!(op_phase.compare(params.EMERGENCY_BRAKING_PHASE) == 0 || op_phase.compare(params.EMERGENCY_STOPPING_PHASE) == 0)){
            op_phase = params.EMERGENCY_BRAKING_PHASE;
        }

    }



    //condition to start ACCELERATING
    if(mobility->getSpeed() > 0 && (op_phase.compare(params.STOPPING_PHASE) == 0 || op_phase.compare(params.EMERGENCY_STOPPING_PHASE) == 0)){

        if(op_phase.compare(params.EMERGENCY_STOPPING_PHASE) == 0){
            traci->vehicle(trainID).setDeccel(params.decel); //set the deceleration back to service braking
        }

        op_phase = params.ACCELERATING_PHASE;
        traci->vehicle(trainID).slowDown(params.Vcru, params.Tcru);
    }

    //condition to start CRUISING
    if(mobility->getSpeed() >= params.Vcru - 1 && op_phase.compare(params.ACCELERATING_PHASE) == 0){
        op_phase = params.CRUISING_PHASE;
        traci->vehicle(trainID).setSpeed(params.Vcru);
    }


    //condition to start DECELERATING
    if(newestWOM.getLMA() - mobility->getPositionAt(simTime()).x <= params.Sdec && op_phase.compare(params.CRUISING_PHASE) == 0){

        if(newestWOM.getLMA() > mobility->getPositionAt(simTime()).x){

            op_phase = params.DECELERATING_PHASE;

            double min_decel_dist = pow(mobility->getSpeed(), 2) / (2 * traci->vehicle(trainID).getDeccel()); //the minimum distance needed for the train to stop currently
            double LMA_decel_dist = newestWOM.getLMA() - mobility->getPositionAt(simTime()).x; //the stopping distance based on the current LMA

            if(LMA_decel_dist < min_decel_dist) {
                traciVehicle->stopAt(params.stopsRoadId, mobility->getPositionAt(simTime()).x + min_decel_dist - params.Xdep, params.stopsLaneId, 100, params.dwellTime);
            }

            else {
                traciVehicle->stopAt(params.stopsRoadId, newestWOM.getLMA() - params.Xdep, params.stopsLaneId, 100, params.dwellTime);
            }

        }

        else {
            //set the operational phase to emergency braking, only if it is not in emergency braking or stopping
            if(!(op_phase.compare(params.EMERGENCY_BRAKING_PHASE) == 0 || op_phase.compare(params.EMERGENCY_STOPPING_PHASE) == 0)){
                op_phase = params.EMERGENCY_BRAKING_PHASE;
            }
        }

    }

    //allows a train to stop for dwell time or emergency stopping time
    if(mobility->getSpeed() <= 1 && (op_phase.compare(params.DECELERATING_PHASE) == 0)){
        traci->vehicle(trainID).setSpeed(-1);
    }


    //condition to start STOPPING
    if(mobility->getSpeed() == 0 && op_phase.compare(params.DECELERATING_PHASE) == 0){
        op_phase = params.STOPPING_PHASE;
    }


    //condition to start emergency braking

        if(op_phase.compare(params.EMERGENCY_BRAKING_PHASE) == 0 && traci->vehicle(trainID).getDeccel() == params.decel){
            traci->vehicle(trainID).setDeccel(params.emergency_decel); //set the deceleration to emergency braking
            double emerg_braking_time = ceil(mobility->getSpeed() / traci->vehicle(trainID).getDeccel());
            traci->vehicle(trainID).slowDown(0, emerg_braking_time);
        }


        //condition to start emergency stopping
        if(mobility->getSpeed() == 0 && op_phase.compare(params.EMERGENCY_BRAKING_PHASE) == 0){

            op_phase = params.EMERGENCY_STOPPING_PHASE;
            traci->vehicle(trainID).setSpeed(-1);
            traci->vehicle(trainID).slowDown(0, params.emergencyStopTime);
        }


    //condition to adjust speed
    if(newestWOM.getAdjustSpeed()){
        double Vcur = mobility->getSpeed();
        double Vdesired = newestWOM.getDesiredSpeed();
        double adjust_speed_time;

        if(Vcur > Vdesired){
            adjust_speed_time = (Vcur - Vdesired) / traci->vehicle(trainID).getDeccel();
        }

        else if(Vdesired > Vcur){
            adjust_speed_time = (Vdesired - Vcur) / traci->vehicle(trainID).getAccel();
        }

        traci->vehicle(trainID).slowDown(Vdesired, adjust_speed_time);
    }


    //condition to start misbehaving by changing speed
    if(op_phase.compare(params.CRUISING_PHASE) == 0 && attackType.compare("CHANGE_SPEED") == 0 && randomNumber() < attackProbability){
        double rand = randomNumber() * (speedChangeMaxPercentage - speedChangeMinPercentage) + speedChangeMinPercentage;
        double currSpeed = mobility->getSpeed();

        int decider = ((randomNumber()*10 + 2) / 1);

        if (decider % 2 == 0){
            traci->vehicle(trainID).slowDown(currSpeed * (1 + (rand/100)), 1);
        }
        else {
            traci->vehicle(trainID).slowDown(currSpeed * (1 - (rand/100)), 1);
        }

    }


    CBTCBaseApplLayer::handlePositionUpdate(obj);

}


void Train11p::setNewestWOM(WaysideOperationalMessage wom){
    newestWOM = wom;
}


void Train11p::finish()
{
    CBTCBaseApplLayer::finish();
}
