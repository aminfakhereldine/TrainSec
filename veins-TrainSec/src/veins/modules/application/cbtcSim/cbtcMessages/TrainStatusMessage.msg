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


cplusplus {{
#include "veins/base/utils/Coord.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
}}

namespace veins;

class BaseFrame1609_4;
class noncobject Coord;

packet TrainStatusMessage extends BaseFrame1609_4 {

    simtime_t sendTime;
    string senderTrainId;
    Coord trainPosition;
    Coord gtTrainPosition; //ground truth train position
    double trainSpeed;
    double gtTrainSpeed; //ground truth train speed
    double trainAcceleration;
    double trainLength;
    double maxSpdStpDist;
    string opPhase;
    string attackType;
}
