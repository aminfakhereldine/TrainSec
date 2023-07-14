//
// Copyright (C) 2016 David Eckhoff <eckhoff@cs.fau.de>
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

#pragma once

#include <map>

#include "veins/base/modules/BaseApplLayer.h"
#include "veins/modules/utility/Consts80211p.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/messages/DemoServiceAdvertisement_m.h"
#include "veins/modules/application/cbtcSim/cbtcMessages/TrainStatusMessage_m.h"
#include "veins/modules/application/cbtcSim/cbtcMessages/WaysideOperationalMessage_m.h"
#include "veins/base/connectionManager/ChannelAccess.h"
#include "veins/modules/mac/ieee80211p/DemoBaseApplLayerToMac1609_4Interface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"



namespace veins {

using veins::AnnotationManager;
using veins::AnnotationManagerAccess;
using veins::TraCICommandInterface;
using veins::TraCIMobility;
using veins::TraCIMobilityAccess;

/**
 * @brief
 * CBTC application layer base class.
 *
 * @author David Eckhoff
 *
 * @ingroup applLayer
 *
 * @see CBTCBaseApplLayer
 * @see Mac1609_4
 * @see PhyLayer80211p
 * @see Decider80211p
 */
class VEINS_API CBTCBaseApplLayer : public BaseApplLayer {

public:
    ~CBTCBaseApplLayer() override;
    void initialize(int stage) override;
    void finish() override;

    void receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details) override;

    enum CBTCApplMessageKinds {
        SEND_TSM_EVT,
        SEND_WOM_EVT,
        SEND_WSA_EVT
    };

protected:
    //generates a random number between 0 and 1
    double randomNumber();

    /** @brief handle messages from below and calls the onWSM, onBSM, and onWSA functions accordingly */
    void handleLowerMsg(cMessage* msg) override;

    /** @brief handle self messages */
    void handleSelfMsg(cMessage* msg) override;

    /** @brief sets all the necessary fields in the TSM, WOM, and WSM, WSA */
    virtual void populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId = LAddress::L2BROADCAST(), int serial = 0);

    /** @brief this function is called upon receiving a BaseFrame1609_4 */
    virtual void onWSM(BaseFrame1609_4* wsm){};

    /** @brief this function is called upon receiving a TrainStatusMessage, similar to a beacon  */
    virtual void onTSM(TrainStatusMessage* tsm){};

    /** @brief this function is called upon receiving a WaysideOperationalMessage, similar to a beacon  */
    virtual void onWOM(WaysideOperationalMessage* wom){};

    /** @brief this function is called upon receiving a DemoServiceAdvertisement */
    virtual void onWSA(DemoServiceAdvertisment* wsa){};

    /** @brief this function is called every time the train receives a position update signal */
    virtual void handlePositionUpdate(cObject* obj);

    /** @brief this function is called every time the train parks or starts moving again */
    virtual void handleParkingUpdate(cObject* obj);

    /** @brief This will start the periodic advertising of the new service on the CCH
     *
     *  @param channel the channel on which the service is provided
     *  @param serviceId a service ID to be used with the service
     *  @param serviceDescription a literal description of the service
     */
    virtual void startService(Channel channel, int serviceId, std::string serviceDescription);

    /** @brief stopping the service and advertising for it */
    virtual void stopService();

    /** @brief compute a point in time that is guaranteed to be in the correct channel interval plus a random offset
     *
     * @param interval the interval length of the periodic message
     * @param chantype the type of channel, either type_CCH or type_SCH
     */
    virtual simtime_t computeAsynchronousSendingTime(simtime_t interval, ChannelType chantype);

    /**
     * @brief overloaded for error handling and stats recording purposes
     *
     * @param msg the message to be sent. Must be a WSM/BSM/WSA; we added TSM and WOM
     */
    virtual void sendDown(cMessage* msg);

    /**
     * @brief overloaded for error handling and stats recording purposes
     *
     * @param msg the message to be sent. Must be a WSM/BSM/WSA; we added TSM and WOM
     * @param delay the delay for the message
     */
    virtual void sendDelayedDown(cMessage* msg, simtime_t delay);

    /**
     * @brief helper function for error handling and stats recording purposes
     *
     * @param msg the message to be checked and tracked
     */
    virtual void checkAndTrackPacket(cMessage* msg);

protected:
    /* pointers ill be set when used with TraCIMobility */
    TraCIMobility* mobility;
    TraCICommandInterface* traci;
    TraCICommandInterface::Vehicle* traciVehicle;

    AnnotationManager* annotations;
    DemoBaseApplLayerToMac1609_4Interface* mac;

    /* support for parking currently only works with TraCI */
    bool isParked;

    /* BSM (beacon) settings */
    uint32_t beaconLengthBits;
    uint32_t beaconUserPriority;
    simtime_t beaconInterval;
    bool sendBeacons;

    /* WSM (data) settings */
    uint32_t dataLengthBits;
    uint32_t dataUserPriority;
    bool dataOnSch;

    std::string moduleType; //determines if a module is a train or a WSU

    //attacks' parameters
    std::string attackType;
    double attackProbability;
    double lowerTSMDelay;
    double upperTSMDelay;
    double lowerWOMDelay;
    double upperWOMDelay;
    double lowerPositionOffset;
    double upperPositionOffset;
    double lowerSpeedOffset;
    double upperSpeedOffset;
    double lowerLMAOffset;
    double upperLMAOffset;
    double speedChangeMinPercentage;
    double speedChangeMaxPercentage;



    /* WSA settings */
    int currentOfferedServiceId;
    std::string currentServiceDescription;
    Channel currentServiceChannel;
    simtime_t wsaInterval;

    /* state of the vehicle */
    Coord curPosition;
    Coord curSpeed;
    LAddress::L2Type myId = 0;
    int mySCH;

    /* stats */
    uint32_t generatedWSMs;
    uint32_t generatedWSAs;
    uint32_t generatedTSMs;
    uint32_t generatedWOMs;
    uint32_t receivedWSMs;
    uint32_t receivedWSAs;
    uint32_t receivedTSMs;
    uint32_t receivedWOMs;
    int generatedEBCs; //generated emergency braking commands

    /* messages for periodic events such as beacon and WSA transmissions */
    cMessage* sendTSMEvt;
    cMessage* sendWOMEvt;
    cMessage* sendWSAEvt;


    std::string wsu_s = "WSU";
    std::string train_s = "Train";

};

} // namespace veins
