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





#ifndef CONSTANTS_HPP
#define CONSTANTS_HPP

const int GPS_SIZE = 32;
const int GLO_SIZE = 24;
const int GAL_SIZE = 30;
const int BDU_SIZE = 34;

const int CONSTELLATION_ID_GPS = 1;
const int CONSTELLATION_ID_GLO = 2;
const int CONSTELLATION_ID_GAL = 3;
const int CONSTELLATION_ID_BDU = 4;

const double TECU = 1.0e16;
const double c = 2.99792458e8;

const double GPS_f1 = 1.57542e+09;
const double GPS_f2 = 1.2276e+09;

const double GLO_f1 = 1.602e+09;
const double GLO_f2 = 1.246e+09;

const double GAL_f1 = 1.57542e+09; //E1  (L1B, L1C, L1X)
const double GAL_f2 = 1.17645e+09; //E5A (L5I, L5Q, L5X)

const double BDU_f1 = 1.561098e+09; //B1  (L1I, L1Q, L1X)
const double BDU_f2 = 1.207140e+09; //B2 (L7I, L7Q, L7X)


const double Glmda1 = 1.0 / GPS_f1;
const double Glmda2 = 1.0 / GPS_f2;

const double Rlmda1 = 1.0 / GLO_f1;
const double Rlmda2 = 1.0 / GLO_f2;

const double Elmda1 = 1.0 / GAL_f1;
const double Elmda2 = 1.0 / GAL_f2;

const double Blmda1 = 1.0 / BDU_f1;
const double Blmda2 = 1.0 / BDU_f2;


const double mu = 3.986005e+14;
const double mu_WGS84 = 3.986004418e+14;
const double wE = 7.2921150e-05;  //Earth's rotation rate in radians per sec.

//Euatorial radius of earth elipsoid in meters (semi-major axis)
const double a_WGS84 = 6378137.0;

//Polar radius of earth elipsoid in meters (semi-minor axis)
const double b_WGS84 = 6356752.314245;


//////////////////////////// PZ-90 Constants ///////////////////////////////////
// Reference: http://www.navipedia.net/index.php/GLONASS_Satellite_Coordinates_Computation
//            By: Hernández-Pajares, Technical University of Catalonia, Spain. 
//
 const double mu_PZ90 = 398600.44; //Gravitational constatnt PZ-90
 const double aE_PZ90 = 6378.136; //Euatorial radius of the earth PZ-90
 const double C20_PZ90 = -1.08263e-03; //Euatorial radius of the earth PZ-90
//
////////////////////////////////////////////////////////////////////////////////

#endif
