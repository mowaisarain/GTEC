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

#include "igrf.hpp"
#include <fstream>
#include <iostream>

std::vector<std::string> igrf::linesplit(std::string str)
{
    std::string field = "";
    char tmp;
    size_t fillcount = 1;
    std::vector<std::string> str_vec;
    for(int i = 0; i < str.length(); ++i)
        {
            tmp = str[i];
            if(tmp == ' ' || tmp == '\t')
                {
                    if(field.length() > 0)
                        {
                            str_vec.push_back(field);
                            field.clear();
                        }
                }
            else
                {
                    field.append(fillcount, tmp);
                }
        }
    str_vec.push_back(field);
    return str_vec;
};



igrf::igrf(int igrf_Year, std::string inpDir)
{
    //Allocate memory
    arraySize = 181 * 360;
    ippI = (double*) malloc( arraySize * sizeof(double) );    
    stationI = (double*) malloc( arraySize * sizeof(double) );
    
    
    std::string fnameIPP;
    std::string fnameStation;

    //set filename
    if(igrf_Year >= 2015 && igrf_Year < 2020)
    {
      fnameIPP = inpDir + "/" + "igrf_2015_IPP.txt";
      fnameStation = inpDir + "/" + "igrf_2015_station.txt";
    }
    else if(igrf_Year >= 2010)
    {
      fnameIPP = inpDir + "/" + "igrf_2010_IPP.txt";
      fnameStation = inpDir + "/" + "igrf_2010_station.txt";
    }
    else if(igrf_Year >= 2005)
    {
      fnameIPP = inpDir + "/" + "igrf_2005_IPP.txt";
      fnameStation = inpDir + "/" + "igrf_2005_station.txt";
    }
    else if(igrf_Year >= 2000)
    {
      fnameIPP = inpDir + "/" + "igrf_2000_IPP.txt";
      fnameStation = inpDir + "/" + "igrf_2000_station.txt";
    }
    else
    {
      std::cout << "Invalid year for IGRF." << std::endl;
      exit(-1);
    }
    
    std::ifstream igrfFileIPP;
    std::ifstream igrfFileStation;

    igrfFileIPP.open(fnameIPP);
    igrfFileStation.open(fnameStation);
    
    std::string line;
    int linecount = 0;
    std::vector<std::string> fields;


    if(igrfFileIPP.is_open())
        {
	    //skip initial 3 lines
	    getline(igrfFileIPP, line);
	    getline(igrfFileIPP, line);
	    getline(igrfFileIPP, line);
	    for (int i=0; i<arraySize; ++i)
	    {
	      getline(igrfFileIPP, line);
	      fields = linesplit(line);
	      ippI[i] = std::stof(fields[3]);
	    }
	}
    else
        {
            std::cout << "Unable to open igrf IPP file\n";
            std::cout << "File Name: " << fnameIPP << std::endl;
            exit(-1);
        }
        
    if(igrfFileStation.is_open())
        {
	    //skip initial 3 lines
	    getline(igrfFileStation, line);
	    getline(igrfFileStation, line);
	    getline(igrfFileStation, line);
	    for (int i=0; i<arraySize; ++i)
	    {
	      getline(igrfFileStation, line);
	      fields = linesplit(line);
	      stationI[i] = std::stof(fields[3]);
	    }
	}
    else
        {
            std::cout << "Unable to open igrf Station file\n";
            std::cout << "File Name: " << fnameStation << std::endl;
            exit(-1);
        }        
        
};



double igrf::getMODIP(const triple& pos)
                      {
                          //get inclination
                          double lat1,lat2,lon1,lon2;
                          lat1 = floor(pos.X);
                          lon1 = floor(pos.Y);
                          lat2 = ceil(pos.X);
                          lon2 = ceil(pos.Y);
                          
                          std::cout << "original: " << pos.X << " " << pos.Y << std::endl;
                          std::cout << "floor: " << lat1 << " " << lon1 << std::endl;
                          std::cout << "ceil: " << lat2 << " " << lon2 << std::endl;
                          std::cout << std::endl;
                      }



//Destructor
igrf::~igrf()
{
    free(ippI);
    free(stationI);
};