//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package org.car2x.veins.modules.application.cbtcSim.base80211p;
import org.car2x.veins.base.modules.IBaseApplLayer;

//
// CBTC application layer base class.
//
// @author David Eckhoff
//
// @see CBTCBaseApplLayer
// @see Nic80211p
// @see Mac1609_4
// @see PhyLayer80211p
//
simple CBTCBaseApplLayer like IBaseApplLayer
{
    parameters:
        @class(veins::CBTCBaseApplLayer);
        int headerLength = default(88bit) @unit(bit); //header length of the application
        
        //the beacon paramters here will be unified for TSMs and WOMs
        //i.e., the beacon can be a TSM if the module is a Train or WOM if the module is WSU
        //customizing values for Trains and WSUs can be done separately
        //the related parameters are: sendBeacons, beaconLengthBits, beaconUserPriority, beaconInterval
        //================================================================================
        bool sendBeacons = default(true); //tells the applayer to periodically send beacons
        int beaconLengthBits = default(256bit) @unit(bit); //the length of a beacon packet
        int beaconUserPriority = default(7); //the user priority (UP) of the beacon messages
        double beaconInterval = default(1s) @unit(s); //the intervall between 2 beacon messages

        int dataLengthBits = default(1024bit) @unit(bit); //the length of a data packet
        bool dataOnSch = default(false); //tells the applayer whether to use a service channel for datapackets or the control channel
        int dataUserPriority = default(7); //the default user priority (UP) for data packets

        bool avoidBeaconSynchronization = default(true); //don't start beaconing directly after node was created but delay to avoid artifical synchronization

        bool sendWSA = default(false);
        int wsaLengthBits = default(250bit) @unit(bit);
        double wsaInterval =  default(1s) @unit(s);
        
        string moduleType; //types are: "WSU" and "Train"
        
        //attacks' parameters
        string attackType;
        double attackProbability = default(0);
        double lowerTSMDelay = default(0);
        double upperTSMDelay = default(0);
        double lowerWOMDelay = default(0);
        double upperWOMDelay = default(0);
        double lowerPositionOffset = default(0);
        double upperPositionOffset = default(0);
        double lowerSpeedOffset = default(0);
        double upperSpeedOffset = default(0);
        double lowerLMAOffset = default(0);
        double upperLMAOffset = default(0);
        double speedChangeMinPercentage = default(0);
        double speedChangeMaxPercentage = default(0);
        
        
        

    gates:
        input lowerLayerIn; // from mac layer
        output lowerLayerOut; // to mac layer
        input lowerControlIn;
        output lowerControlOut;
}