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



#include "ephemerisR.hpp"
#include <cmath>
#include "triple.hpp"
#include "constants.hpp"

void ephemerisR::position(int h, triple &pos)
{

	double w10 = px;
	double w20 = py;
	double w30 = pz;
	double w40 = vx;
	double w50 = vy;
	double w60 = vz;
	
	double r = sqrt(px*px + py*py + pz*pz);
    double r2 = r*r;
    double r3 = r2*r;
    double r4 = r3*r;
    double r5 = r4*r;
    
    double mu_r3 = -1 * mu_PZ90 / r3;
    double c20mu = 1.5 * C20_PZ90 * mu_PZ90 * aE_PZ90 * aE_PZ90 / r5;
    double w30_2 = w30 * w30;
    double w30_2_r2 = w30_2 / r2;
    double w30_2_r2_5 = w30_2_r2 * 5.0;
    double w30_2_15 = 1.0 - w30_2_r2_5;
    double w3_2 = wE*wE;
    double w3t2 = wE * 2.0;

	double k11 = h * w40;
	double k12 = h * w50;
	double k13 = h * w60;
    
	double k14 = h*((mu_r3 * w10 )+(c20mu *w10)*(w30_2_15)+ w3_2 * w10 + w3t2 * w50 + xdd);
	double k15 = h*((mu_r3 * w20 )+(c20mu *w20)*(w30_2_15)+ w3_2 * w20 - w3t2 * w40 + ydd);
	double k16 = h*((mu_r3 * w30 )+(c20mu *w30)*(3.0 - w30_2_r2_5)+ zdd);
	
//    std::cout << "k11: " << k11 << " k12: " << k12 << " k13: " << k13
//              << " k14: " << k14 << " k15: " << k15 << " k16: " << k16 << "\n";

	double k21 = h*(w40 + 0.5 * k14);
	double k22 = h*(w50 + 0.5 * k15);
	double k23 = h*(w60 + 0.5 * k16);
    
    double w10k11 = w10 + 0.5 * k11;
    double w20k12 = w20 + 0.5 * k12;
    double w30k13 = w30 + 0.5 * k13;
    double w30k13_5r = w30k13 * w30k13 * 5.0 / r2;
    double w30k13_1 = 1.0 - w30k13_5r;
    double w30k13_3 = 3.0 - w30k13_5r;

    
	double k24 = h*((mu_r3 * w10k11)+(c20mu * w10k11  *w30k13_1)+ w3_2 * w10k11 + w3t2 * (w50 + 0.5 * k15) + xdd);
	double k25 = h*((mu_r3 * w20k12)+(c20mu * w20k12  *w30k13_1)+ w3_2 * w20k12 - w3t2 * (w40 + 0.5 * k14) + ydd);
	double k26 = h*((mu_r3 * w30k13)+(c20mu * w30k13  *w30k13_3)+ zdd);


//    std::cout << "k21: " << k21 << " k22: " << k22 << " k23: " << k23
//              << " k24: " << k24 << " k25: " << k25 << " k26: " << k26 << "\n";


	double k31 = h*(w40 + 0.5 * k24);
	double k32 = h*(w50 + 0.5 * k25);
	double k33 = h*(w60 + 0.5 * k26);
    
    double w10k21 = w10 + 0.5 * k21;
    double w20k22 = w20 + 0.5 * k22;
    double w30k23 = w30 + 0.5 * k23;
    
    double w30k23_5r = w30k23 * w30k23 * 5.0 / r2;
    double w30k23_1 = 1.0 - w30k23_5r;
    double w30k23_3 = 3.0 - w30k23_5r;
    
	double k34 = h*((mu_r3 * w10k21)+(c20mu * w10k21 * w30k23_1)+ w3_2 * w10k21 + w3t2 * (w50 + 0.5 * k25) + xdd);
	double k35 = h*((mu_r3 * w20k22)+(c20mu * w20k22 * w30k23_1)+ w3_2 * w20k22 - w3t2 * (w40 + 0.5 * k24) + ydd);
	double k36 = h*((mu_r3 * w30k23)+(c20mu * w30k23 * w30k23_3)+ zdd);


//    std::cout << "k31: " << k31 << " k32: " << k32 << " k33: " << k33
//              << " k34: " << k34 << " k35: " << k35 << " k36: " << k36 << "\n";



	double k41 = h*(w40 + k34);
	double k42 = h*(w50 + k35);
	double k43 = h*(w60 + k36);
    
    double w10k31 = w10 + k31;
    double w20k32 = w20 + k32;
    double w30k33 = w30 + k33;
    
    double w30k33_5r = w30k33 * w30k33 * 5.0 / r2;
    double w30k33_1 = 1.0 - w30k33_5r;
    double w30k33_3 = 3.0 - w30k33_5r;
    
	double k44 = h*((mu_r3 * w10k31)+(c20mu * w10k31 * w30k33_1)+ w3_2 * w10k31 + w3t2 * (w50 + k35) + xdd);
	double k45 = h*((mu_r3 * w20k32)+(c20mu * w20k32 * w30k33_1)+ w3_2 * w20k32 - w3t2 * (w40 + k34) + ydd);
	double k46 = h*((mu_r3 * w30k33)+(c20mu * w30k33 * w30k33_3)+ zdd);


//    std::cout << "k41: " << k41 << " k42: " << k42 << " k43: " << k43
//              << " k44: " << k44 << " k45: " << k45 << " k46: " << k46 << "\n";

    double onesixth = 1.0/6.0;
	pos.X = w10 + onesixth*(k11 + 2.0*k21 + 2.0*k31 + k41);
	pos.Y = w20 + onesixth*(k12 + 2.0*k22 + 2.0*k32 + k42);
	pos.Z = w30 + onesixth*(k13 + 2.0*k23 + 2.0*k33 + k43);
//	rxd = w40 + onesixth*(k14 + 2.0*k24 + 2.0*k34 + k44);
//	ryd = w50 + onesixth*(k15 + 2.0*k25 + 2.0*k35 + k45);
//	rzd = w60 + onesixth*(k16 + 2.0*k26 + 2.0*k36 + k46);
    
//    std::cout << rx << "  " << ry << "  " << rz << "\n";

};


