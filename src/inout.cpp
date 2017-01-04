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

	std::ifstream inpfile;
	inpfile.open(args[1]);
    
    
    if (inpfile.is_open())
    {
        while(std::getline(inpfile, line))
        {
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
                std::cout << line << std::endl;
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

	s << "Number of Obs Files: " << nobsfiles << "\n";
	s << "Number of Nav Files: " << nnavfiles << "\n";
	s << "System string: " << "\n";
	s << "GPS: " << systemGPS << "\n";
	s << "GLONASS: " << systemGlonass << "\n";
	s << "Galileo: " << systemGalileo << "\n";
	s << "Beidou: " << systemBeidou << "\n";
	s << "QZSS: " << systemQZSS << "\n";

};












