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
#include "inout.hpp"
#include <string>
#include <sstream>
#include <algorithm>
#include <exception>




inout::inout()
{
    navfiles = {""};
    obsfiles = {""};
    bool systemGPS = false;
    bool systemGlonass = false;
    bool systemGalileo = false;
    bool systemBeidou = false;
    bool systemQZSS = false;
};



//Process Inputs
void inout::process_Inputs(int ac, char* args[])
{
    
    if(ac != 2)
    {
        std::cout << "Usage: " << args[0] << " <config-file>" << std::endl;
        exit(1);
    }

    std::string line;
    std::string parameter;
    std::string value;
    int lineNumber=0;

    std::ifstream inpfile;
    inpfile.open(args[1]);
    
    
    if (inpfile.is_open())
    {
        while(std::getline(inpfile, line))
        {
            lineNumber += 1;
            
            //strip comment
            line.erase( std::find( line.begin(), line.end(), '#' ), line.end() );
            //strip spaces
            line.erase(remove_if(line.begin(), line.end(), isspace), line.end());
            
            if(line.size() == 0)
            {
                //Continue to next line
                continue;
            }
            else
            {
                //process line
                parameter = line.substr(0,line.find( '=' ));
                
                if (parameter == "NUMDAYS")
                {
                    //Set number of days to calibrate
                    value = line.substr(line.find( '=' )+1);
                    try
                    {
                        numDays = stoi(value);
                    }
                    catch (std::exception& e)
                    {
                        std::cout << "Invalid parameter value in config file at line: " << lineNumber << "\n";
                        exit(1);
                    }
                }
                else if (parameter == "INPDIR")
                {
                    //Set input directory
                    inputDirectory = line.substr(line.find( '=' )+1);
                }
                else if (parameter == "SATSYS")
                {
                    //Set satellite systems string
                    satSys = line.substr(line.find( '=' )+1);
                }
                else if (parameter == "SAMPLINGTIME")
                {
                    //Set sampling time
                    value = line.substr(line.find( '=' )+1);
                    try
                    {
                        sampingTime = stoi(value);
                    }
                    catch (std::exception& e)
                    {
                        std::cout << "Invalid parameter value in config file at line: " << lineNumber << "\n";
                        exit(1);
                    }
                }
                else if (parameter == "MINARCLEN")
                {
                    //Set sampling time
                    value = line.substr(line.find( '=' )+1);
                    try
                    {
                        minArcLen = stoi(value);
                    }
                    catch (std::exception& e)
                    {
                        std::cout << "Invalid parameter value in config file at line: " << lineNumber << "\n";
                        exit(1);
                    }
                }
                else if (parameter == "INTRPOLINTRVL")
                {
                    //Set sampling time
                    value = line.substr(line.find( '=' )+1);
                    try
                    {
                        intrpolIntrvl = stoi(value);
                    }
                    catch (std::exception& e)
                    {
                        std::cout << "Invalid parameter value in config file at line: " << lineNumber << "\n";
                        exit(1);
                    }
                }
                else if (parameter == "DEGREE")
                {
                    //Set sampling time
                    value = line.substr(line.find( '=' )+1);
                    try
                    {
                        deg = stoi(value);
                    }
                    catch (std::exception& e)
                    {
                        std::cout << "Invalid parameter value in config file at line: " << lineNumber << "\n";
                        exit(1);
                    }
                }
                else
                {
                    //Invalid parameter
                    std::cout << "Invalid parameter in config file at line: " << lineNumber << "\n";
                    exit(1);
                }
                
            }
        }
    }
    else
    {
        std::cout << "Cannot open config file.\n";
        exit(1);
    }
    
    
    inpfile.close();
};


void inout::dump(std::ostream& s)
{
    s << "System Configuration from config file.\n";
    s << "---------------------------------------\n";
    s << "Number of Days: " << numDays << "\n";
    s << "Input Directory: " << inputDirectory << "\n";
    s << "System string: " << satSys << "\n";
    s << "Sampling Time: " << sampingTime << "\n";
    s << "Minimum Arc Length: " << minArcLen << "\n";
    s << "Interpolation Interval: " << intrpolIntrvl << "\n";
    s << "Interpolation Degree: " << deg << "\n";
    s << "---------------------------------------";
    s << std::endl;
};












