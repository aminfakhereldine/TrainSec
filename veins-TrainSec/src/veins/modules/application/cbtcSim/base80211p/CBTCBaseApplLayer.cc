//
// Copyright (C) 2011 David Eckhoff <eckhoff@cs.fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#include "veins/modules/application/cbtcSim/base80211p/CBTCBaseApplLayer.h"

using namespace veins;

void CBTCBaseApplLayer::initialize(int stage)
{
    BaseApplLayer::initialize(stage);

    if (stage == 0) {

        // initialize pointers to other modules
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = nullptr;
            mobility = nullptr;
            traciVehicle = nullptr;
        }

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<DemoBaseApplLayerToMac1609_4Interface*>::findSubModule(getParentModule());
        ASSERT(mac);

        // read parameters
        headerLength = par("headerLength");
        sendBeacons = par("sendBeacons").boolValue();
        beaconLengthBits = par("beaconLengthBits");
        beaconUserPriority = par("beaconUserPriority");
        beaconInterval = par("beaconInterval");

        dataLengthBits = par("dataLengthBits");
        dataOnSch = par("dataOnSch").boolValue();
        dataUserPriority = par("dataUserPriority");

        wsaInterval = par("wsaInterval").doubleValue();
        currentOfferedServiceId = -1;

        moduleType = par("moduleType").stdstringValue(); //determines if a module is a train or a WSU

        //attacks' parameters
        attackType = par("attackType").stdstringValue();
        attackProbability = par("attackProbability");
        lowerTSMDelay = par("lowerTSMDelay");
        upperTSMDelay = par("upperTSMDelay");
        lowerWOMDelay = par("lowerWOMDelay");
        upperWOMDelay = par("upperWOMDelay");
        lowerPositionOffset = par("lowerPositionOffset");
        upperPositionOffset = par("upperPositionOffset");
        lowerSpeedOffset = par("lowerSpeedOffset");
        upperSpeedOffset = par("upperSpeedOffset");
        lowerLMAOffset = par("lowerLMAOffset");
        upperLMAOffset = par("upperLMAOffset");
        speedChangeMinPercentage = par("speedChangeMinPercentage");
        speedChangeMaxPercentage = par("speedChangeMaxPercentage");



        isParked = false;

        findHost()->subscribe(BaseMobility::mobilityStateChangedSignal, this);
        findHost()->subscribe(TraCIMobility::parkingStateChangedSignal, this);

        sendTSMEvt = new cMessage("tsm evt", SEND_TSM_EVT);
        sendWOMEvt = new cMessage("wom evt", SEND_WOM_EVT);
        sendWSAEvt = new cMessage("wsa evt", SEND_WSA_EVT);


        generatedTSMs = 0;
        generatedWOMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        receivedTSMs = 0;
        receivedWOMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;

        generatedEBCs = 0;

    }
    else if (stage == 1) {

        // store MAC address for quick access
        myId = mac->getMACAddress();

        // simulate asynchronous channel access

        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            EV_ERROR << "App wants to send data on SCH but MAC doesn't use any SCH. Sending all data on CCH" << std::endl;
        }
        simtime_t firstBeacon = simTime();

        if (par("avoidBeaconSynchronization").boolValue() == true) {

            simtime_t randomOffset = dblrand() * beaconInterval;
            firstBeacon = simTime() + randomOffset;

            if (mac->isChannelSwitchingActive() == true) {
                if (beaconInterval.raw() % (mac->getSwitchingInterval().raw() * 2)) {
                    EV_ERROR << "The beacon interval (" << beaconInterval << ") is smaller than or not a multiple of  one synchronization interval (" << 2 * mac->getSwitchingInterval() << "). This means that beacons are generated during SCH intervals" << std::endl;
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, ChannelType::control);
            }

            if (sendBeacons) {

                if (moduleType.compare(train_s) == 0) {
                    scheduleAt(firstBeacon, sendTSMEvt);
                }


                else if(moduleType.compare(wsu_s) == 0) {
                    scheduleAt(firstBeacon, sendWOMEvt);
                }
            }


        }
    }
}

simtime_t CBTCBaseApplLayer::computeAsynchronousSendingTime(simtime_t interval, ChannelType chan)
{

    /*
     * avoid that periodic messages for one channel type are scheduled in the other channel interval
     * when alternate access is enabled in the MAC
     */

    simtime_t randomOffset = dblrand() * interval;
    simtime_t firstEvent;
    simtime_t switchingInterval = mac->getSwitchingInterval(); // usually 0.050s
    simtime_t nextCCH;

    /*
     * start event earliest in next CCH (or SCH) interval. For alignment, first find the next CCH interval
     * To find out next CCH, go back to start of current interval and add two or one intervals
     * depending on type of current interval
     */

    if (mac->isCurrentChannelCCH()) {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval * 2;
    }
    else {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval;
    }

    firstEvent = nextCCH + randomOffset;

    // check if firstEvent lies within the correct interval and, if not, move to previous interval

    if (firstEvent.raw() % (2 * switchingInterval.raw()) > switchingInterval.raw()) {
        // firstEvent is within a sch interval
        if (chan == ChannelType::control) firstEvent -= switchingInterval;
    }
    else {
        // firstEvent is within a cch interval, so adjust for SCH messages
        if (chan == ChannelType::service) firstEvent += switchingInterval;
    }

    return firstEvent;
}

void CBTCBaseApplLayer::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial)
{
    wsm->setRecipientAddress(rcvId);
    wsm->setBitLength(headerLength);

    if (TrainStatusMessage* tsm = dynamic_cast<TrainStatusMessage*>(wsm)) {
        tsm->setPsid(-1);
        tsm->setChannelNumber(static_cast<int>(Channel::cch));
        tsm->addBitLength(beaconLengthBits);
        tsm->setUserPriority(beaconUserPriority);
    }


    else if (WaysideOperationalMessage* wom = dynamic_cast<WaysideOperationalMessage*>(wsm)) {
        wom->setSenderWSUPosition(curPosition);
        wom->setPsid(-1);
        wom->setChannelNumber(static_cast<int>(Channel::cch));
        wom->addBitLength(beaconLengthBits);
        wom->setUserPriority(beaconUserPriority);
    }

    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(static_cast<int>(Channel::cch));
        wsa->setTargetChannel(static_cast<int>(currentServiceChannel));
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    else {
        if (dataOnSch)
            wsm->setChannelNumber(static_cast<int>(Channel::sch1)); // will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        else
            wsm->setChannelNumber(static_cast<int>(Channel::cch));
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
    }
}

void CBTCBaseApplLayer::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    Enter_Method_Silent();
    if (signalID == BaseMobility::mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == TraCIMobility::parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void CBTCBaseApplLayer::handlePositionUpdate(cObject* obj)
{
    ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getPositionAt(simTime());
    curSpeed = mobility->getCurrentSpeed();
}

void CBTCBaseApplLayer::handleParkingUpdate(cObject* obj)
{
    isParked = mobility->getParkingState();
}


double CBTCBaseApplLayer::randomNumber(){
    return static_cast<double>(std::rand()) / RAND_MAX;
}

void CBTCBaseApplLayer::handleLowerMsg(cMessage* msg)
{

    BaseFrame1609_4* wsm = dynamic_cast<BaseFrame1609_4*>(msg);
    ASSERT(wsm);

    if (TrainStatusMessage* tsm = dynamic_cast<TrainStatusMessage*>(wsm)) {
        receivedTSMs++;
        onTSM(tsm);
    }

    else if (WaysideOperationalMessage* wom = dynamic_cast<WaysideOperationalMessage*>(wsm)) {
        receivedWOMs++;
        onWOM(wom);
    }

    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        receivedWSAs++;
        onWSA(wsa);
    }

    else {
        receivedWSMs++;
        onWSM(wsm);
    }

    delete (msg);
}

void CBTCBaseApplLayer::handleSelfMsg(cMessage* msg)
{
    switch (msg->getKind()) {

    case SEND_WSA_EVT: {
        DemoServiceAdvertisment* wsa = new DemoServiceAdvertisment();
        populateWSM(wsa);
        sendDown(wsa);
        scheduleAt(simTime() + wsaInterval, sendWSAEvt);
        break;
    }

    default: {
        if (msg) EV_WARN << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
        break;
    }

    }

}

void CBTCBaseApplLayer::finish()
{
    recordScalar("generatedWSMs", generatedWSMs);
    recordScalar("receivedWSMs", receivedWSMs);

    recordScalar("generatedTSMs", generatedTSMs);
    recordScalar("receivedTSMs", receivedTSMs);

    recordScalar("generatedWOMs", generatedWOMs);
    recordScalar("receivedWOMs", receivedWOMs);

    recordScalar("generatedWSAs", generatedWSAs);
    recordScalar("receivedWSAs", receivedWSAs);

    recordScalar("generatedEBCs", generatedEBCs);

}

CBTCBaseApplLayer::~CBTCBaseApplLayer()
{
    cancelAndDelete(sendTSMEvt);
    cancelAndDelete(sendWOMEvt);
    cancelAndDelete(sendWSAEvt);

    findHost()->unsubscribe(BaseMobility::mobilityStateChangedSignal, this);
}

void CBTCBaseApplLayer::startService(Channel channel, int serviceId, std::string serviceDescription)
{
    if (sendWSAEvt->isScheduled()) {
        throw cRuntimeError("Starting service although another service was already started");
    }

    mac->changeServiceChannel(channel);
    currentOfferedServiceId = serviceId;
    currentServiceChannel = channel;
    currentServiceDescription = serviceDescription;

    simtime_t wsaTime = computeAsynchronousSendingTime(wsaInterval, ChannelType::control);
    scheduleAt(wsaTime, sendWSAEvt);
}

void CBTCBaseApplLayer::stopService()
{
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

void CBTCBaseApplLayer::sendDown(cMessage* msg)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void CBTCBaseApplLayer::sendDelayedDown(cMessage* msg, simtime_t delay)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void CBTCBaseApplLayer::checkAndTrackPacket(cMessage* msg)
{
    if (dynamic_cast<TrainStatusMessage*>(msg)) {
        EV_TRACE << "sending down a TSM" << std::endl;
        generatedTSMs++;
    }

    else if (dynamic_cast<WaysideOperationalMessage*>(msg)) {
        EV_TRACE << "sending down a WOM" << std::endl;
        generatedWOMs++;
    }

    else if (dynamic_cast<DemoServiceAdvertisment*>(msg)) {
        EV_TRACE << "sending down a WSA" << std::endl;
        generatedWSAs++;
    }
    else if (dynamic_cast<BaseFrame1609_4*>(msg)) {
        EV_TRACE << "sending down a wsm" << std::endl;
        generatedWSMs++;
    }

}
