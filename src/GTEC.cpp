/*
    GTEC -  A high performance standardized implementation of 
    Multi Constellation GNSS Derived TEC Calibration 
    (Model by T/ICT4D Lab ICTP).
    Copyright (C) 2016,2017  Muhammad Owais
    
    This file is part of GTEC.

    GTEC is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, version 2 of the License.

    GTEC is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with GTEC.  If not, see <http://www.gnu.org/licenses/>.
    
    Disclaimer: GTEC is a research implementation which is under 
    development and should not be considered fully functional unless 
    otherwise stated or a release is announced. Author is providing this 
    software on a best effort AS IS basis and do not warrant validity, 
    functionality, and suitability for any particular purpose. All copyright 
    notices must be kept intact. 

*/



#include <iostream>
#include <fstream>
#include "triple.hpp"
#include "constants.hpp"
#include "navigation.hpp"
#include "inout.hpp"
#include "ObsData.hpp"
#include "solver.hpp"



int main(int argc, char* argv[])
{    
    
    inout io;
    io.process_Inputs(argc,argv);
    io.dump(std::cout);

    int samplingtime = 10; //in Minutes
    
    ObsData obs(io.obsfiles,io.satSys);
    obs.read();
    
    navigation navdata(io.navfiles);
    navdata.read();
    
    std::cout << "done reading all files..\n\n";

    
    std::cout << "preprocessing..\n";
    obs.pre_process(io.minArcLen,io.intrpolIntrvl,io.deg);

    int rejHours = 12; //Number of hours to reject from start and end
    int arcHours = 2; //Duration of arc in hours
    obs.markArcStartEnd(rejHours, arcHours);
    std::cout << "done preprocessing..\n\n";


    //Create solver object
    solver sys(obs, navdata);

    
    //building S
    std::cout << "building S..\n";
    sys.buildS(io.samplingTime);
    std::cout << "Size of S: " << obs.S.size() << " Number of Arcs: " << obs.numArcs << "\n";
    std::cout << "Done building S..\n\n";


    //building B
    std::cout << "building B..\n";
    sys.buildB();
    std::cout << "Done building B..\n";

    
    
    sys.cleanUp();




    
    return 0;
}



















