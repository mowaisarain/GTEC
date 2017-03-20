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
#include <boost/filesystem.hpp>

using namespace boost::filesystem;

inout::inout()
{
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
                    //numDays should be between 1 and 3
                    if(numDays < 1 || numDays > 3)
                    {
                        std::cout << "Invalid parameter value in config file at line: " << lineNumber << "\n";
                        std::cout << "Valid range for NUMDAYS is between 1 and 3.\n";
                        exit(1);
                    }
                }
                else if (parameter == "FIRSTDOY")
                {
                    //Set first day of year
                    value = line.substr(line.find( '=' )+1);
                    try
                    {
                        firstDayOfYear = stoi(value);
                    }
                    catch (std::exception& e)
                    {
                        std::cout << "Invalid parameter value in config file at line: " << lineNumber << "\n";
                        exit(1);
                    }
                }
                else if (parameter == "YEAR")
                {
                    //Set first day of year
                    value = line.substr(line.find( '=' )+1);
                    try
                    {
                        year = stoi(value);
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
                        samplingTime = stoi(value);
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
                else if (parameter == "MARKER")
                {
                    //Set marker (station) name
                    marker = line.substr(line.find( '=' )+1);
                }
                else if (parameter == "NUMCOEFFS")
                {
                    value = line.substr(line.find( '=' )+1);
                    //Set numCoeffs
                    try
                    {
                        numCoeffs = stoi(value);
                    }
                    catch (std::exception& e)
                    {
                        std::cout << "Invalid parameter value in config file at line: " << lineNumber << "\n";
                        exit(1);
                    }
                    //numCoeffs should be 6 or 9
                    if(numCoeffs != 6 || numCoeffs != 9)
                    {
                        std::cout << "Invalid parameter value in config file at line: " << lineNumber << "\n";
                        std::cout << "Valid value for NUMCOEFFS is 6 or 9.\n";
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
    
    
    checkInputFiles();
    
    
};



void inout::checkInputFiles()
{
    //list files in input directory
    path p(inputDirectory);
    path filePath("");
    std::string pathString;
    std::size_t found;
    std::string tmp;
    
    int dayStart;
    int dayEnd;
    int fileDay;
    
    
    if(firstDayOfYear == 1)
    {
        dayStart = 365;
    }
    else
    {
        dayStart = firstDayOfYear - 1;
    }
    
    dayEnd = firstDayOfYear + numDays;
    
    
    if(exists(p))
    {
        if (is_directory(p))
        {
            //List files
            std::cout << "Input directory path: " << p << std::endl;
            for (directory_entry& d: directory_iterator(p))
            {
                filePath = d.path();
                pathString = filePath.string();
                
                
                found = pathString.find(marker);
                if (found != std::string::npos)
                {
                    //check days in file name
                    tmp = pathString.substr(found);
                    if (tmp.size() == 12)
                    {
                        fileDay = stoi(tmp.substr(4, 3));
                        if (fileDay >= dayStart && fileDay <= dayEnd)
                        {
                            obsfiles.push_back(pathString);
                        }
                    }
                    else if (tmp.size() == 38)
                    {
                        std::cout << tmp << "  " << tmp.size() << "\n";
                        fileDay = stoi(tmp.substr(16, 3));        //std::cout << "<<<<< " << i <<  " >>>>>>>>\n";

                        std::cout << fileDay << "\n";
                        if (fileDay >= dayStart && fileDay <= dayEnd)
                        {
                            obsfiles.push_back(pathString);
                        }
                    }
                }
                else
                {
                    found = pathString.find("brdm");
                    if (found != std::string::npos)
                    {
                        fileDay = stoi(pathString.substr(found+4, 3));
                        if (fileDay >= dayStart && fileDay <= dayEnd)
                        {
                            navfiles.push_back(pathString);
                        }
                    }
                }
            }
            //Sort file names
            std::sort(obsfiles.begin(),obsfiles.end());
            std::sort(navfiles.begin(),navfiles.end());
        }
        else
        {
            //Path not a directory!
            std::cout << "Input path not a directory.\n";
            exit(1);
        }
    }
    else
    {
        //Path does not exist!
        std::cout << "Input directory doesn't exist.\n";
        exit(1);
    }
    
    
    //Check if enough files are found!
    
    if ( (obsfiles.size() != (numDays + 2)) || (navfiles.size() != (numDays + 2)) )
    {
        std::cout << "Not enough obs/nav files in input directory.\n";
        exit(1);
    }
    
};



void inout::dump(std::ostream& s)
{
    s << "System Configuration from config file.\n";
    s << "---------------------------------------\n";
    s << "Number of Days: " << numDays << "\n";
    s << "Year: " << year << "\n";
    s << "First day of year: " << firstDayOfYear << "\n";
    s << "Input Directory: " << inputDirectory << "\n";
    s << "System string: " << satSys << "\n";
    s << "Sampling Time: " << samplingTime << "\n";
    s << "Minimum Arc Length: " << minArcLen << "\n";
    s << "Interpolation Interval: " << intrpolIntrvl << "\n";
    s << "Interpolation Degree: " << deg << "\n";
    s << "Marker Name: " << marker << "\n";
    s << "Observation Files:\n";
    for (auto file: obsfiles)
    {
        s << file << "\n";
    }
    s << "Navigation Files:\n";
    for (auto file: navfiles)
    {
        s << file << "\n";
    }
    s << "---------------------------------------";
    s << std::endl;
};












