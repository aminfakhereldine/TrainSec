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

import org.car2x.veins.base.connectionManager.ConnectionManager;
import org.car2x.veins.base.modules.BaseWorldUtility;
import org.car2x.veins.modules.mobility.traci.TraCIScenarioManager*;
import org.car2x.veins.modules.world.annotations.AnnotationManager;
import org.car2x.veins.visualizer.roads.RoadsCanvasVisualizer;
import org.car2x.veins.modules.application.cbtcSim.nodes.Train;
import org.car2x.veins.modules.application.cbtcSim.nodes.WSU;
import org.car2x.veins.modules.application.cbtcSim.nodes.Station;




network CBTCSim1
{
    parameters:
        double playgroundSizeX @unit(m); // x size of the area the nodes are in (in meters)
        double playgroundSizeY @unit(m); // y size of the area the nodes are in (in meters)
        double playgroundSizeZ @unit(m); // z size of the area the nodes are in (in meters)
        @display("bgb=$playgroundSizeX,$playgroundSizeY");
    submodules:

        annotations: AnnotationManager {
            @display("p=250,2500");
        }
        
        connectionManager: ConnectionManager {
            parameters:
                @display("p=150,2500;i=abstract/multicast");
        }
        
        world: BaseWorldUtility {
            parameters:
                playgroundSizeX = playgroundSizeX;
                playgroundSizeY = playgroundSizeY;
                playgroundSizeZ = playgroundSizeZ;
                @display("p=50,2500;i=misc/globe");
        }
        
        manager: TraCIScenarioManagerLaunchd {
            parameters:
                @display("p=500,2500");
        }
        
        roadsCanvasVisualizer: RoadsCanvasVisualizer {
            @display("p=300,2500");
        }

		node[0]: Train {
		     @display("i=veins/node/train;is=s");
        }
        
        station[9]: Station {
            @display("p=250,50;i=veins/trainstation");
        }
        
        wsu[9]: WSU {
            @display("p=50,50;i=device/antennatower;is=s");
        }
        


    connections allowunconnected:
        
}
