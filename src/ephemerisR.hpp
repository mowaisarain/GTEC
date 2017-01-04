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




#ifndef __EPHEMERISR__
#define __EPHEMERISR__

#include "triple.hpp"


class ephemerisR
{
    public:
        int tb;     //Ephemerides reference epoch 
        
        float px;   //Coordinate at te, in PZ-90 
        float py;   //Coordinate at te, in PZ-90
        float pz;   //Coordinate at te, in PZ-90
        
        float vx;   //Velocity component at te, in PZ-90
        float vy;   //Velocity component at te, in PZ-90
        float vz;   //Velocity component at te, in PZ-90
        
        float xdd;  //Sun and Moon acceleration at te
        float ydd;  //Sun and Moon acceleration at te
        float zdd;  //Sun and Moon acceleration at te
        
        void position(int , triple&);
    
};

#endif