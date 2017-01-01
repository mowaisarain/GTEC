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

	std::string buffer;

	std::ifstream inpfile;
	inpfile.open(args[1]);


	std::getline(inpfile, buffer);
	obsfiles[0] = buffer;
	nobsfiles = atoi(buffer.c_str());

	std::getline(inpfile, buffer);
	navfiles[0] = buffer;
	nnavfiles = atoi(buffer.c_str());

	for(int i=0;i<nobsfiles;++i)
	{
		std::getline(inpfile, buffer);
		obsfiles.push_back(buffer);
	}
	for(int i=0;i<nnavfiles;++i)
	{
		std::getline(inpfile, buffer);
		navfiles.push_back(buffer);
	}

	//Now process last two lines of input parameters
	std::getline(inpfile, buffer);
	for(int i=0; i<buffer.size(); ++i)
	{
		if(buffer[i] == 'G')
			systemGPS = true;
		if(buffer[i] == 'R')
			systemGlonass = true;
		if(buffer[i] == 'E')
			systemGalileo = true;
		if(buffer[i] == 'B')
			systemBeidou = true;
		if(buffer[i] == 'Q')
			systemQZSS = true;
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












