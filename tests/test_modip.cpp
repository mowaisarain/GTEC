/*
    GTEC -  A high performance standardized implementation of 
    Multi Constellation GNSS Derived TEC Calibration 
    (Model by T/ICT4D Lab ICTP).
    Copyright (C) 2016,2017 Muhammad Owais
    
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

#include "triple.hpp"
#include "igrf.hpp"


int main()
{
    //Reference point Trieste-Italy
    //Latitude 45° 38' 42" N   Longitude 13° 46' 37" E (in Deg,Min,Sec)
    //Latitude 45.645°       Longitude 13.77694°   (in Decimal Degrees)
    //Elevation/Height      350.0 km   Mean Sea Level
    
    //Calculated Inclination from online model
    //https://www.ngdc.noaa.gov/geomag-web/?model=igrf#igrfwmm
    //Inclination = 61° 30' 11" or 61.50306°
    //MODIP = atan(I / sqrt(cos(lat))) == I = 
    
    //create position object
    triple pos(45.645, 13.77694, 350.0);
    
    //set time in unit of Years
    int t = 2017;
    
    
    
}