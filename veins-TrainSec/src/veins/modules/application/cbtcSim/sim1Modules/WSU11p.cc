#include "veins/modules/application/cbtcSim/sim1Modules/WSU11p.h"

#include <iostream>
#include <fstream>

#include "veins/base/phyLayer/PhyToMacControlInfo.h"
#include "veins/modules/phy/DeciderResult80211.h"

using namespace veins;
using namespace std;

Define_Module(veins::WSU11p);

//WSU's initialization
void WSU11p::initialize(int stage)
{
    //initializing WSU
    CBTCBaseApplLayer::initialize(stage);

    if (stage == 0) {

        //create wsu ID
        wsuID = "wsu_" + std::to_string(this->getParentModule()->getIndex());
        index = this->getParentModule()->getIndex();



        if(params.generateDatasets){
            //add the features' names if the dataset file didn't exist before
            std::string filename = "tsmFile.csv";
            std::ifstream file(filename);

            if(!file){
                ofstream tsmFile;
                tsmFile.open("tsmFile.csv",ios::app);
                assert(!tsmFile.fail());
                tsmFile << "tripID,rcvTime,sendTime,trainID,posx,gtPosx,spd,gtSpd,accl,opPhase,rssi,attackType" << endl;
                tsmFile.close();
                assert(!tsmFile.fail());
            }

        }

    }

}


//WSU's behavior upon receiving a TrainStatusMessage (TSM) from a train
void WSU11p::onTSM(TrainStatusMessage* tsm)
{

    double rssi = check_and_cast<DeciderResult80211*>(check_and_cast<PhyToMacControlInfo*>(tsm->getControlInfo())->getDeciderResult())->getRecvPower_dBm();


    if(params.generateDatasets){
        ofstream tsmFile;
        tsmFile.open("tsmFile.csv",ios::app);
        assert(!tsmFile.fail());


        tsmFile << params.tripID << "," << simTime().dbl() << "," << tsm->getSendTime().dbl() << "," << tsm->getSenderTrainId() << "," << tsm->getTrainPosition().x << ","
                << tsm->getGtTrainPosition().x << "," << tsm->getTrainSpeed() << "," << tsm->getGtTrainSpeed() << ","<< tsm->getTrainAcceleration() << ","
                << tsm->getOpPhase() << "," << rssi << "," << tsm->getAttackType() << endl;

        tsmFile.close();
        assert(!tsmFile.fail());
    }

    storeTSM(*tsm);

}


void WSU11p::handleSelfMsg(cMessage* msg)
{

    //populate and send a WOM
    switch (msg->getKind()) {


    case SEND_WOM_EVT: {

        if(tsmsBeforeWSU.size() > 0){
            sortTSMs(tsmsBeforeWSU);

            for (int i=0; i<tsmsBeforeWSU.size()-1; i++) {
                WaysideOperationalMessage* wom = new WaysideOperationalMessage();
                wom->setSendTime(simTime());
                wom->setSenderWSUId(wsuID.c_str());
                wom->setReceiverTrainId(tsmsBeforeWSU[i].getSenderTrainId());
                wom->setAttackType("NORMAL");


                //computing LMA
                if(attackType.compare("RANDOM_LMA_OFFSET") == 0 && randomNumber() < attackProbability) {
                    double randomOffset = randomNumber() * (upperLMAOffset - lowerLMAOffset) + lowerLMAOffset;
                    wom->setLMA(tsmsBeforeWSU[i].getTrainPosition().x + distToPrecTrain(tsmsBeforeWSU[i], tsmsBeforeWSU[i+1]) - params.safetyMargin + randomOffset);
                    wom->setAttackType(attackType.c_str());
                }

                else {
                    wom->setLMA(tsmsBeforeWSU[i].getTrainPosition().x + distToPrecTrain(tsmsBeforeWSU[i], tsmsBeforeWSU[i+1]) - params.safetyMargin);
                }


                //LMA ground truth
                wom->setGtLMA(tsmsBeforeWSU[i].getTrainPosition().x + distToPrecTrain(tsmsBeforeWSU[i], tsmsBeforeWSU[i+1]) - params.safetyMargin);


                //checking safe separation
                if(!areSafelySeparated(tsmsBeforeWSU[i], tsmsBeforeWSU[i+1])){
                    wom->setApplyEmergencyBraking(true);
                    generatedEBCs++;
                }

                //checking overspeed protection
                if(checkSpeedStatus(tsmsBeforeWSU[i]) == speedStatus::dangerous){
                    wom->setApplyEmergencyBraking(true);
                    generatedEBCs++;
                }

                else if(checkSpeedStatus(tsmsBeforeWSU[i]) == speedStatus::overspeeding || checkSpeedStatus(tsmsBeforeWSU[i]) == speedStatus::downspeeding){
                    wom->setAdjustSpeed(true);
                    wom->setDesiredSpeed(params.Vcru);
                }

                populateWSM(wom);


                if(attackType.compare("RANDOM_WOM_DELAY") == 0 && randomNumber() < attackProbability) {
                    double randomDelay = randomNumber() * (upperWOMDelay - lowerWOMDelay) + lowerWOMDelay;
                    wom->setAttackType(attackType.c_str());
                    sendDelayedDown(wom, randomDelay);
                }

                else{
                    sendDown(wom);
                }

            }

            //closest train to the station: LMA is the station's location
            WaysideOperationalMessage* wom = new WaysideOperationalMessage();
            wom->setSendTime(simTime());
            wom->setSenderWSUId(wsuID.c_str());
            wom->setReceiverTrainId(tsmsBeforeWSU[tsmsBeforeWSU.size()-1].getSenderTrainId());
            wom->setAttackType("NORMAL");

            //computing LMA
            if(attackType.compare("RANDOM_LMA_OFFSET") == 0 && randomNumber() < attackProbability) {
                double randomOffset = randomNumber() * (upperLMAOffset - lowerLMAOffset) + lowerLMAOffset;
                wom->setLMA(params.stationsLocs[index] + randomOffset);
                wom->setAttackType(attackType.c_str());
            }

            else {
                wom->setLMA(params.stationsLocs[index]);
            }


            //LMA ground truth
            wom->setGtLMA(params.stationsLocs[index]);


            //checking overspeed protection
            if(checkSpeedStatus(tsmsBeforeWSU[tsmsBeforeWSU.size()-1]) == speedStatus::dangerous){
                wom->setApplyEmergencyBraking(true);
                generatedEBCs++;
            }

            else if(checkSpeedStatus(tsmsBeforeWSU[tsmsBeforeWSU.size()-1]) == speedStatus::overspeeding || checkSpeedStatus(tsmsBeforeWSU[tsmsBeforeWSU.size()-1]) == speedStatus::downspeeding){
                wom->setAdjustSpeed(true);
                wom->setDesiredSpeed(params.Vcru);
            }

            populateWSM(wom);



            if(attackType.compare("RANDOM_WOM_DELAY") == 0 && randomNumber() < attackProbability) {
                double randomDelay = randomNumber() * (upperWOMDelay - lowerWOMDelay) + lowerWOMDelay;
                wom->setAttackType(attackType.c_str());
                sendDelayedDown(wom, randomDelay);
            }

            else{
                sendDown(wom);
            }

        }


        if(tsmsAfterWSU.size() > 0){
            sortTSMs(tsmsAfterWSU);

            for (int i=0; i<tsmsAfterWSU.size()-1; i++) {
                WaysideOperationalMessage* wom = new WaysideOperationalMessage();
                wom->setSendTime(simTime());
                wom->setSenderWSUId(wsuID.c_str());
                wom->setReceiverTrainId(tsmsAfterWSU[i].getSenderTrainId());
                wom->setAttackType("NORMAL");

                //computing LMA
                if(attackType.compare("RANDOM_LMA_OFFSET") == 0 && randomNumber() < attackProbability) {
                    double randomOffset = randomNumber() * (upperLMAOffset - lowerLMAOffset) + lowerLMAOffset;
                    wom->setLMA(tsmsAfterWSU[i].getTrainPosition().x + distToPrecTrain(tsmsAfterWSU[i], tsmsAfterWSU[i+1]) - params.safetyMargin + randomOffset);
                    wom->setAttackType(attackType.c_str());
                }

                else{
                    wom->setLMA(tsmsAfterWSU[i].getTrainPosition().x + distToPrecTrain(tsmsAfterWSU[i], tsmsAfterWSU[i+1]) - params.safetyMargin);
                }


                //LMA ground truth
                wom->setGtLMA(tsmsAfterWSU[i].getTrainPosition().x + distToPrecTrain(tsmsAfterWSU[i], tsmsAfterWSU[i+1]) - params.safetyMargin);


                //checking safe separation
                if(!areSafelySeparated(tsmsAfterWSU[i], tsmsAfterWSU[i+1])){
                    wom->setApplyEmergencyBraking(true);
                    generatedEBCs++;
                }


                //checking overspeed protection
                if(checkSpeedStatus(tsmsAfterWSU[i]) == speedStatus::dangerous){
                    wom->setApplyEmergencyBraking(true);
                    generatedEBCs++;
                }

                else if(checkSpeedStatus(tsmsAfterWSU[i]) == speedStatus::overspeeding || checkSpeedStatus(tsmsAfterWSU[i]) == speedStatus::downspeeding){
                    wom->setAdjustSpeed(true);
                    wom->setDesiredSpeed(params.Vcru);
                }

                populateWSM(wom);



                if(attackType.compare("RANDOM_WOM_DELAY") == 0 && randomNumber() < attackProbability) {
                    double randomDelay = randomNumber() * (upperWOMDelay - lowerWOMDelay) + lowerWOMDelay;
                    wom->setAttackType(attackType.c_str());
                    sendDelayedDown(wom, randomDelay);
                }

                else{
                    sendDown(wom);
                }


            }

            //closest train to the next station: LMA is the next station's location
            WaysideOperationalMessage* wom = new WaysideOperationalMessage();
            wom->setSendTime(simTime());
            wom->setSenderWSUId(wsuID.c_str());
            wom->setReceiverTrainId(tsmsAfterWSU[tsmsAfterWSU.size()-1].getSenderTrainId());
            wom->setAttackType("NORMAL");

            //computing LMA
            if(index == params.stationsLocs.size()-1){ //reached end of track
                wom->setLMA(-1);
                wom->setReachedEOT(true);

                //LMA ground truth
                wom->setGtLMA(-1);
            } else {

                if(attackType.compare("RANDOM_LMA_OFFSET") == 0 && randomNumber() < attackProbability) {
                    double randomOffset = randomNumber() * (upperLMAOffset - lowerLMAOffset) + lowerLMAOffset;
                    wom->setLMA(params.stationsLocs[index+1] + randomOffset);
                    wom->setAttackType(attackType.c_str());
                }

                else{
                    wom->setLMA(params.stationsLocs[index+1]);
                }

                //LMA ground truth
                wom->setGtLMA(params.stationsLocs[index+1]);


            }

            //checking overspeed protection
            if(checkSpeedStatus(tsmsAfterWSU[tsmsAfterWSU.size()-1]) == speedStatus::dangerous){
                wom->setApplyEmergencyBraking(true);
                generatedEBCs++;
            }

            else if(checkSpeedStatus(tsmsAfterWSU[tsmsAfterWSU.size()-1]) == speedStatus::overspeeding || checkSpeedStatus(tsmsAfterWSU[tsmsAfterWSU.size()-1]) == speedStatus::downspeeding){
                wom->setAdjustSpeed(true);
                wom->setDesiredSpeed(params.Vcru);
            }

            populateWSM(wom);



            if(attackType.compare("RANDOM_WOM_DELAY") == 0 && randomNumber() < attackProbability) {
                double randomDelay = randomNumber() * (upperWOMDelay - lowerWOMDelay) + lowerWOMDelay;
                wom->setAttackType(attackType.c_str());
                sendDelayedDown(wom, randomDelay);
            }

            else{
                sendDown(wom);
            }

        }


        //clear messages after sending WOMs
        tsmsBeforeWSU.clear();
        tsmsAfterWSU.clear();

        scheduleAt(simTime() + beaconInterval, sendWOMEvt);
        break;
    }


    default: {
        CBTCBaseApplLayer::handleSelfMsg(msg);
        break;
    }

    }

}

void WSU11p::storeTSM(TrainStatusMessage tsm){
    if(tsm.getTrainPosition().x < params.stationsLocs[index]){
        tsmsBeforeWSU.push_back(tsm);
    }

    else{
        tsmsAfterWSU.push_back(tsm);
    }
}

//sorts the TSMs in ascending order of position
void WSU11p::sortTSMs(std::vector<TrainStatusMessage>& tsms){
    int i, j, n;
    n = tsms.size();

    for (i = 0; i < n - 1; i++){

        for (j = 0; j < n - i - 1; j++) {

            if (tsms[j].getTrainPosition().x > tsms[j + 1].getTrainPosition().x) {
                std::swap(tsms[j], tsms[j + 1]);
            }

        }

    }
}


//returns the distance between train T1 and its preceding train T2
//computed as: (distance from front of T1 to front of T2) - (length of T2)
double WSU11p::distToPrecTrain(TrainStatusMessage& tsm_T1, TrainStatusMessage& tsm_T2){
    double trains_fronts_separation = sqrt(tsm_T1.getTrainPosition().sqrdist(tsm_T2.getTrainPosition()));
    return trains_fronts_separation - tsm_T2.getTrainLength();
}


//checks safe separation:
bool WSU11p::areSafelySeparated(TrainStatusMessage& tsm_T1, TrainStatusMessage& tsm_T2){
    double safe_sep_thr = tsm_T1.getMaxSpdStpDist() + params.safetyMargin; //safe separation threshold
    if(distToPrecTrain(tsm_T1, tsm_T2) <= safe_sep_thr){
        return false;
    } else {
        return true;
    }
}

//checks speed status for overspeed protection:
speedStatus WSU11p::checkSpeedStatus(TrainStatusMessage& tsm){
    //checks in the cruising phase because the speed is always lower in the other phases
    std::string op_phase = tsm.getOpPhase();
    if(op_phase.compare(params.CRUISING_PHASE) == 0){
        double speed = tsm.getTrainSpeed();
        double accp_spd_lwr = params.Vcru * (1 - (params.acceptable_spd_limit / 100.0)); //lower limit for acceptable speed
        double accp_spd_upr = params.Vcru * (1 + (params.acceptable_spd_limit / 100.0)); //upper limit for acceptable speed
        double over_spd_upr = params.Vcru * (1 + (params.overspeed_limit / 100.0)); //upper limit for overspeeding (lower limit for dangerous speeding)

        if(speed < accp_spd_lwr) {
            return speedStatus::downspeeding;
        }

        else if(speed >= accp_spd_lwr && speed <= accp_spd_upr){
            return speedStatus::acceptable;
        }

        else if(speed > accp_spd_upr && speed < over_spd_upr){
            return speedStatus::overspeeding;
        }

        else {
            return speedStatus::dangerous;
        }
    }

}


void WSU11p::finish()
{
    CBTCBaseApplLayer::finish();
}

