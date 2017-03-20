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




#ifndef INOUT_HPP
#define INOUT_HPP

#include <string>
#include <vector>

class inout
{
  public:
	int nobsfiles;		//Number of Input Observation Files
	int nnavfiles;		//Number of Input Navigation Files
    
	bool systemGPS;
	bool systemGlonass;
	bool systemGalileo;
	bool systemBeidou;
	bool systemQZSS;
    
    int numDays;
    std::string inputDirectory;
    std::string satSys;
    std::string marker;
    int samplingTime;
    int firstDayOfYear;
    int year;
    
    
    int minArcLen;
    int intrpolIntrvl;
    int deg;
    int numCoeffs;
    

    //Observation file names from imput directory
	std::vector<std::string> obsfiles;	 
    
    //Navigation file names from imput directory
	std::vector<std::string> navfiles;	


	void process_Inputs(int ac, char* args[]);
	inout();
	void dump(std::ostream& s);
    
    
  private:
    void checkInputFiles();

};



#endif
