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






#ifndef __EPHEMERISGE__
#define __EPHEMERISGE__

#include "triple.hpp"


class ephemerisGE
{
    public:
        int Toc;        //Time of clock (GPS/GAL) converted to UNIX time
        int Toe;        //Ephemeris reference epoch in seconds with in GPS/GAL week
        int week;       //GPS/GAL week # (to go with Toe)   
        float Ahalf;    //Square root of semi-major axis 
        float e;        //Ecentricity
        float M0;       //mean anomaly at reference epoch
        float w;        //argument of perigee
        float i0;       //inclination at reference epoch
        float Omega0;   //Longitude of ascending node at the begining of the week
        float deltan;   //mean motion difference
        float idot;     //Rate of inclination angle
        float Omegadot; //Rate of node's right ascension
        float Cuc;      //Latitude Argument correction
        float Cus;      //Latitude Argument correction
        float Crc;      //Orbital radius correction
        float Crs;      //Orbital radius correction
        float Cic;      //Inclination correction
        float Cis;      //Inclination correction
        
        void position(int, triple&);
        
    private:
        float eccAnomaly(float, float);
        void applyRotations(float&, float&, float&, float&, triple&);


};

#endif