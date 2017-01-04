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




#include "ephemerisGE.hpp"
#include "constants.hpp"
#include <cmath>

void ephemerisGE::position(int t, triple& pos)
{
    // Input t --> is current time in GPS seconds for which postion is to be calculated
    // Output pos --> is the resultant position vector (X,Y,Z) (pos.X,posY,posZ)

    // Compute the time tk from the ephemerides reference epoch Toe (t and Toe are expressed in seconds in the GPS week)
    int tk = t - Toe;

    if(tk > 302400) {
        tk = tk - 604800;
    } else if(tk < -302400) {
        tk = tk + 604800;
    }

    // Compute the mean anomaly Mk for tk
    // Here mu = 3.986005e+14 defined in constants source file
    //     M0 is mean anomaly at refernce epoch (fron nav epoch record)
    //     Ahalf is sqrt of semi-major axis (fron nav epoch record)
    float Mk = M0 + (sqrt(mu) / (Ahalf * Ahalf * Ahalf) + deltan) * tk;

    // Solve (iteratively) the Kepler equation for the eccentricity anomaly Ek
    // Arguments Mk from previous step
    //          e eccentricity fron nav epoch record
    float Ek = eccAnomaly(Mk, e);

    // Compute the true anomaly vk
    // using Ek computed in previous step and e eccentricity fron nav epoch record
    float vk = atan((sqrt(1 - e * e) * sin(Ek)) / (cos(Ek) - e));

    // Compute the argument of latitude
    // from: the argument of perigee (w) (fron nav epoch record),
    //       true anomaly vk (computed in previous step),
    //       and corrections Cuc and Cus (fron nav epoch record):
    float uk = w + vk + (Cuc * cos(2 * (w + vk))) + (Cus * sin(2 * (w + vk)));

    // Compute the radial distance rk,
    // considering: corrections Crc and Crs (fron nav epoch record):
    float rk = (Ahalf * Ahalf) * (1 - e * cos(Ek)) + (Crc * cos(2 * (w + vk))) + (Crs * sin(2 * (w + vk)));

    // Compute the inclination of the orbital plane
    // from: the inclination i0 (i0) (fron nav epoch record) at reference time Toe,
    // and corrections Cic and Cis (fron nav epoch record):
    float ik = i0 + (tk * idot) + (Cic * cos(2 * (w + vk))) + (Cis * sin(2 * (w + vk)));

    // Compute the longitude of the ascending node LAMBDAk (Lk) (with respect to Greenwich). This calculation uses:
    // the right ascension at the beginning of the current week (Omega0) (fron nav epoch record),
    // the correction from the apparent sidereal time variation in Greenwich between the beginning of the week and
    // reference time tk,
    // and the change in longitude of the ascending node from the reference time Toe (Toe fron nav epoch record):
    // wE is Earth's rotation rate in radians per sec defined in constants source file.
    // OMEGAdot is rate of node's right ascension (fron nav epoch record)
    float Lk = Omega0 + ((Omegadot - wE) * tk) - (wE * Toe);

    // Compute the coordinates in TRS frame, applying three rotations (around uk, ik and Lk ):
    applyRotations(Lk, ik, uk, rk, pos);
};



// This Routine Calculates eccentricity anomaly Ek
// by Solving (iteratively) the Kepler equation for the eccentricity anomaly, 
// using Newton–Raphson method, Euation -->  Mk = Ek - ( e * Sin(Ek) )
float ephemerisGE::eccAnomaly(float M, float e)
{
    // Input M --> mean anomaly for reference time tk
    // Input e --> eccentricity from nav epoch record
    // return Output --> eccentricity anomaly Ek

    // Initial Guess for Ek
    // We can also use M(Mk) as initial guess ??
    float Ek0 = 0.5;

    // F --> function f(Ek)
    float F = 0.0;

    // Fp --> derivative of function f(Ek)
    float Fp = 0.0;

    // Estimated value of Ek
    float Ek = 0.0;

    // Absolute Relative Error as stoping criteria for iterations
    float Rerr = 0.0;

    do {
        // calculate function at initial guess
        F = Ek0 - (e * sin(Ek0)) - M;
        // calculate function derivative at initial guess
        Fp = 1 - (e * cos(Ek0));
        // Estimate Ek
        Ek = Ek0 - (F / Fp);
        // Calculate Absolute Relative Error
        Rerr = abs(((Ek - Ek0) / Ek) * 100.0);
        // save estimate of Ek to be used for next Iteration
        Ek0 = Ek;
    }
    // Check if required precision is achieved
    while(Rerr > 0.00005);

    // Return Final Result Ek
    return Ek;
};


// This routine apply rotations around uk, ik and Lk
void ephemerisGE::applyRotations(float& Lk, float& ik, float& uk, float& rk, triple& pos)
{
    // Rotation ==
    //  | Xk |                           | rk |
    //  | Yk |  =  R3(-Lk)R1(-ik)R3(-uk) | 0  |
    //  | Zk |                           | 0  |

    // Wwhere R1 and R3 are the rotation matrices defined at:
    // http://www.navipedia.net/index.php/Transformation_between_Terrestrial_Frames
    // By Hernández-Pajares, Technical University of Catalonia, Spain.

    // Initialize Rotation Matrices with zeros
    float R3Lk[3][3] = { 0.0 };
    float R1ik[3][3] = { 0.0 };
    float R3uk[3][3] = { 0.0 };

    // Initialize Temp Matrices
    float Res1[3][3] = { 0.0 };
    float Res2[3][3] = { 0.0 };

    // Initialization of vector Rk
    float Rk[3] = { rk, 0.0, 0.0 };

    // Vector to hold results
    float Res[3] = { 0.0 };

    // Loop Variables
    int i, j, k;

    // Definition of Matrix R3(Lk)
    R3Lk[0][0] = cos(-1 * Lk);
    R3Lk[0][1] = sin(-1 * Lk);
    R3Lk[1][0] = -1 * sin(-1 * Lk);
    R3Lk[1][1] = cos(-1 * Lk);
    R3Lk[2][2] = 1.0;

    // Definition of Matrix R1(ik)
    R1ik[0][0] = 1.0;
    R1ik[1][1] = cos(-1 * ik);
    R1ik[1][2] = sin(-1 * ik);
    R1ik[2][1] = -1 * sin(-1 * ik);
    R1ik[2][2] = cos(-1 * ik);

    // Definition of Matrix R3(uk)
    R3uk[0][0] = cos(-1 * uk);
    R3uk[0][1] = sin(-1 * uk);
    R3uk[1][0] = -1 * sin(-1 * uk);
    R3uk[1][1] = cos(-1 * uk);
    R3uk[2][2] = 1.0;

    // Apply Rotations by multiplying matrices
    for(i = 0; i < 3; i++) {
        for(j = 0; j < 3; j++) {
            for(k = 0; k < 3; k++) {
                Res1[i][j] += R3Lk[i][k] * R1ik[k][j];
            }
        }
    }
    for(i = 0; i < 3; i++) {
        for(j = 0; j < 3; j++) {
            for(k = 0; k < 3; k++) {
                Res2[i][j] += Res1[i][k] * R3uk[k][j];
            }
        }
    }
    for(i = 0; i < 3; i++) {
        for(j = 0; j < 3; j++) {
            Res[i] += Res2[i][j] * Rk[j];
        }
    }

    // Store resultant positions
    pos.X = Res[0];
    pos.Y = Res[1];
    pos.Z = Res[2];
};


