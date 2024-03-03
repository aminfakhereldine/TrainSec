# TrainSec

This project presents TrainSec, a simulation framework that facilitates security modeling and evaluation in networks of Communication-Based Train Control (CBTC) systems. The framework simulates components and communications in CBTC networks according to [IEEE 1474.1](https://doi.org/10.1109/IEEESTD.2004.95746), the standard for CBTC performance and functional requirements. 



A scientific publication is [available](https://link.springer.com/chapter/10.1007/978-3-031-43366-5_2) for TrainSec. If you find it useful in your research work, please consider citing as follows:

```bibtex
@inproceedings{fakhereldine2023trainsec,
  title={TrainSec: A simulation framework for security modeling and evaluation in CBTC networks},
  author={Fakhereldine, Amin and Zulkernine, Mohammad and Murdock, Dan},
  booktitle={International Conference on Reliability, Safety, and Security of Railway Systems},
  pages={22--39},
  year={2023},
  organization={Springer}
}

```

## Features

- Modeling cyber attacks and misbehavior algorithms in CBTC networks.
- Implementing detection and mitigation algorithms.
- Utilizing the provided misbehavior algorithms.
- Evaluating the performance and effectiveness of the implemented algorithms.
- Extracting datasets for data visualization purposes.


## TrainSec Framework Architecture
<img src="TrainSec%20framework%20architecture.png" alt="TrainSec framework architecture" width="681" height="1152">

Following is a mapping of the framework components to their corresponding files and folders in the code:

- **TrainSec components:** veins-TrainSec\src\veins\modules\application\cbtcSim
- **VEINS components\SUMO-Scenario Manager:** veins-TrainSec\cbtc-simulations\cbtc-sim1\cbtc-sim1.launchd.xml
- **VEINS components\OMNET_NetworkConfigurator:** veins-TrainSec\cbtc-simulations\cbtc-sim1\omnetpp.ini


## Installation
- [Download SUMO](https://sourceforge.net/projects/sumo/files/sumo/) (Recommended version: 1.9.2).
- [Download and build OMNeT++](https://omnetpp.org/download/) (Recommended version: 5.6.2).
- [Download and build Veins](https://veins.car2x.org/download/) (Recommended version: 5.1).
- Replace the Veins project by the provided ***veins-TrainSec*** project, or place the framework architecture components in their corresponding directories as explained above.

### Useful resources related to installation:
- [Veins installation tutorial](https://veins.car2x.org/tutorial/)
- [Getting started with OMNET++, INET, Veins, and SUMO by Dr. Joanne Skiles](https://www.youtube.com/watch?v=PfAWhrmoYgM)
