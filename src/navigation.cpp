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
#include <string>
#include <fstream>
#include <sstream>
#include <exception>
#include <cmath>

#include "navigation.hpp"
#include "internalTime.hpp"
#include "constants.hpp"

navigation::navigation(std::vector<std::string> fnames)
{
    // set filenames
    fileNames = fnames;

    // allocate data structure
    ephemeris_G = { {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {} };
    ephemeris_R = { {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {} };
    ephemeris_E = { {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {} };
    ephemeris_C = { {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {},
                    {} };
};

void navigation::read()
{
    std::ifstream navFile;

    std::string line;
    std::string str;
    std::size_t pos;
    internalTime epoch_time;
    ephemerisGE datum_GE;
    ephemerisR datum_R;

    int lineNumber = 0;
    int linecount = 0;
    int prn;

    for(auto fname : fileNames) {
        navFile.open(fname);

        if(navFile.is_open()) {
            while(std::getline(navFile, line)) {
                ++lineNumber;
                if(line[60] == 'R' && line[66] == 'V' && line[76] == 'T') {
                    // VERSION
                    version = stof(line);
                    continue;
                }
                if(line[60] == 'L' && line[63] == 'P' && line[71] == 'S') {
                    // LEAP SECONDS
                    leapSeconds = stoi(line);
                    continue;
                }
                if(line[60] == 'E' && line[64] == 'O' && line[67] == 'H') {
                    // END OF HEADER
                    break;
                }
            } // End of HEADER parsing

            // Start reading data records
            while(!navFile.eof()) {
                std::getline(navFile, line);
                ++lineNumber;

                if(line[0] == 'G') {
                    try {
                        linecount = 0;
                        // GPS record started
                        str = line.substr(1, 3);
                        prn = stoi(str);
                        // get epoch time in UNIX
                        str = line.substr(3);
                        epoch_time.year = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.month = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.day = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.hour = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.minute = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.second = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.toUNIXTime();

                        // check if this is a duplicate record
                        if(ephemeris_G[prn - 1].size() != 0)
                            if(ephemeris_G[prn - 1].back().Toc == epoch_time.UNIX) {
                                // Do not process this record and
                                // skip all lines ORBIT - 1 to 7
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                continue;
                            }

                        // set Toc
                        datum_GE.Toc = epoch_time.UNIX;

                        // Next Line ORBIT - 1
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.Crs = stof(line, &pos); // IODE not required!
                        str = line.substr(pos);
                        datum_GE.Crs = stof(str, &pos); // Crs (meters)
                        str = str.substr(pos);
                        datum_GE.deltan = stof(str, &pos); // Delta n (radians/sec)
                        str = str.substr(pos);
                        datum_GE.M0 = stof(str, &pos); // M0 (radians)

                        // Next Line ORBIT - 2
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.Cuc = stof(line, &pos); // Cuc (radians)
                        str = line.substr(pos);
                        datum_GE.e = stof(str, &pos); // e Eccentricity
                        str = str.substr(pos);
                        datum_GE.Cus = stof(str, &pos); // Cus (radians)
                        str = str.substr(pos);
                        datum_GE.Ahalf = stof(str, &pos); // sqrt(A)

                        // Next Line ORBIT - 3
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.Toe = stof(line, &pos); // Time of Ephemeris (sec of GPS week)
                        str = line.substr(pos);
                        datum_GE.Cic = stof(str, &pos); // Cic (radians)
                        str = str.substr(pos);
                        datum_GE.Omega0 = stof(str, &pos); // OMEGA0 (radians)
                        str = str.substr(pos);
                        datum_GE.Cis = stof(str, &pos); // Cis (radians)

                        // Next Line ORBIT - 4
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.i0 = stof(line, &pos); // Time of Ephemeris (sec of GPS week)
                        str = line.substr(pos);
                        datum_GE.Crc = stof(str, &pos); // Cic (radians)
                        str = str.substr(pos);
                        datum_GE.w = stof(str, &pos); // omega argument of perigee (radians)
                        str = str.substr(pos);
                        datum_GE.Omegadot = stof(str, &pos); // OMEGA DOT (radians/sec)

                        // Next Line ORBIT - 5
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.idot = stof(line, &pos); // IDOT (radians/sec)
                        str = line.substr(pos);
                        datum_GE.week = stof(str, &pos); // codes on L2 (Ignore)
                        str = str.substr(pos);
                        datum_GE.week = stof(str, &pos); // GPS/GAL week # (to go with Toe)

                        // Now datum_GE is complete push based on prn
                        ephemeris_G[prn - 1].push_back(datum_GE);

                        // Skip remaining lines
                        // Next Line ORBIT - 6
                        std::getline(navFile, line);
                        ++linecount;
                        // Next Line ORBIT - 7
                        std::getline(navFile, line);
                        ++linecount;
                    } catch(std::exception& e) {
                        // cannot process this record, skip lines
                        while(linecount != 7) {
                            std::getline(navFile, line);
                            ++linecount;
                        }
                    }

                } else if(line[0] == 'R') {
                    try {
                        linecount = 0;
                        // GLONASS record started
                        str = line.substr(1, 3);
                        prn = stoi(str);

                        if(prn > 24) {
                            // Do not process this record and
                            // skip all lines
                            std::getline(navFile, line);
                            std::getline(navFile, line);
                            std::getline(navFile, line);
                            continue;
                        }

                        // get epoch time in UNIX
                        str = line.substr(3);
                        epoch_time.year = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.month = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.day = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.hour = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.minute = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.second = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.toUNIXTime();

                        // Push reference epoch
                        datum_R.tb = epoch_time.UNIX;

                        // Next Line ORBIT - 1
                        std::getline(navFile, line);
                        ++linecount;
                        datum_R.px = stof(line, &pos); // X Coordinate at te, in PZ-90 (Km)
                        str = line.substr(pos);
                        datum_R.vx = stof(str, &pos); // Velocity X component at te, in PZ-90 (Km / sec)
                        str = str.substr(pos);
                        datum_R.xdd = stof(str, &pos); // Sun and Moon acceleration X at te (Km / sec2)

                        // Next Line ORBIT - 2
                        std::getline(navFile, line);
                        ++linecount;
                        datum_R.py = stof(line, &pos); // Y Coordinate at te, in PZ-90 (Km)
                        str = line.substr(pos);
                        datum_R.vy = stof(str, &pos); // Velocity Y component at te, in PZ-90 (Km / sec)
                        str = str.substr(pos);
                        datum_R.ydd = stof(str, &pos); // Sun and Moon acceleration Y at te (Km / sec2)

                        // Next Line ORBIT - 3
                        ++linecount;
                        std::getline(navFile, line);
                        datum_R.pz = stof(line, &pos); // Z Coordinate at te, in PZ-90 (Km)
                        str = line.substr(pos);
                        datum_R.vz = stof(str, &pos); // Velocity Z component at te, in PZ-90 (Km / sec)
                        str = str.substr(pos);
                        datum_R.zdd = stof(str, &pos); // Sun and Moon acceleration Z at te (Km / sec2)
                        // Push datum
                        ephemeris_R[prn - 1].push_back(datum_R);
                    } catch(std::exception& e) {
                        // cannot process this record, skip lines
                        while(linecount != 3) {
                            std::getline(navFile, line);
                            ++linecount;
                        }
                    }

                } else if(line[0] == 'E') {
                    try {
                        linecount = 0;
                        // GALILEO record started
                        str = line.substr(1, 3);
                        prn = stoi(str);
                        // get epoch time in UNIX
                        str = line.substr(3);
                        epoch_time.year = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.month = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.day = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.hour = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.minute = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.second = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.toUNIXTime();

                        // check if this is a duplicate record
                        if(ephemeris_E[prn - 1].size() != 0)
                            if(ephemeris_E[prn - 1].back().Toc == epoch_time.UNIX) {
                                // Do not process this record and
                                // skip all lines ORBIT - 1 to 7
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                continue;
                            }

                        // set Toc
                        datum_GE.Toc = epoch_time.UNIX;

                        // Next Line ORBIT - 1
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.Crs = stof(line, &pos); // IODE not required!
                        str = line.substr(pos);
                        datum_GE.Crs = stof(str, &pos); // Crs (meters)
                        str = str.substr(pos);
                        datum_GE.deltan = stof(str, &pos); // Delta n (radians/sec)
                        str = str.substr(pos);
                        datum_GE.M0 = stof(str, &pos); // M0 (radians)

                        // Next Line ORBIT - 2
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.Cuc = stof(line, &pos); // Cuc (radians)
                        str = line.substr(pos);
                        datum_GE.e = stof(str, &pos); // e Eccentricity
                        str = str.substr(pos);
                        datum_GE.Cus = stof(str, &pos); // Cus (radians)
                        str = str.substr(pos);
                        datum_GE.Ahalf = stof(str, &pos); // sqrt(A)

                        // Next Line ORBIT - 3
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.Toe = stof(line, &pos); // Time of Ephemeris (sec of GPS week)
                        str = line.substr(pos);
                        datum_GE.Cic = stof(str, &pos); // Cic (radians)
                        str = str.substr(pos);
                        datum_GE.Omega0 = stof(str, &pos); // OMEGA0 (radians)
                        str = str.substr(pos);
                        datum_GE.Cis = stof(str, &pos); // Cis (radians)

                        // Next Line ORBIT - 4
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.i0 = stof(line, &pos); // Time of Ephemeris (sec of GPS week)
                        str = line.substr(pos);
                        datum_GE.Crc = stof(str, &pos); // Cic (radians)
                        str = str.substr(pos);
                        datum_GE.w = stof(str, &pos); // omega argument of perigee (radians)
                        str = str.substr(pos);
                        datum_GE.Omegadot = stof(str, &pos); // OMEGA DOT (radians/sec)

                        // Next Line ORBIT - 5
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.idot = stof(line, &pos); // IDOT (radians/sec)
                        str = line.substr(pos);
                        datum_GE.week = stof(str, &pos); // codes on L2 (Ignore)
                        str = str.substr(pos);
                        datum_GE.week = stof(str, &pos); // GPS/GAL week # (to go with Toe)

                        // Now datum_GE is complete push based on prn
                        ephemeris_E[prn - 1].push_back(datum_GE);

                        // Skip remaining lines
                        // Next Line ORBIT - 6
                        std::getline(navFile, line);
                        ++linecount;
                        // Next Line ORBIT - 7
                        std::getline(navFile, line);
                        ++linecount;
                    } catch(std::exception& e) {
                        // cannot process this record, skip lines
                        while(linecount != 7) {
                            std::getline(navFile, line);
                            ++linecount;
                        }
                    }

                } else if(line[0] == 'C') {
                    try {
                        linecount = 0;
                        // BEIDOU record started
                        str = line.substr(1, 3);
                        prn = stoi(str);
                        // get epoch time in UNIX
                        str = line.substr(3);
                        epoch_time.year = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.month = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.day = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.hour = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.minute = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.second = stoi(str, &pos);
                        str = str.substr(pos);

                        epoch_time.toUNIXTime();

                        // check if this is a duplicate record
                        if(ephemeris_C[prn - 1].size() != 0)
                            if(ephemeris_C[prn - 1].back().Toc == epoch_time.UNIX) {
                                // Do not process this record and
                                // skip all lines ORBIT - 1 to 7
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                std::getline(navFile, line);
                                continue;
                            }

                        // set Toc
                        datum_GE.Toc = epoch_time.UNIX;

                        // Next Line ORBIT - 1
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.Crs = stof(line, &pos); // IODE not required!
                        str = line.substr(pos);
                        datum_GE.Crs = stof(str, &pos); // Crs (meters)
                        str = str.substr(pos);
                        datum_GE.deltan = stof(str, &pos); // Delta n (radians/sec)
                        str = str.substr(pos);
                        datum_GE.M0 = stof(str, &pos); // M0 (radians)

                        // Next Line ORBIT - 2
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.Cuc = stof(line, &pos); // Cuc (radians)
                        str = line.substr(pos);
                        datum_GE.e = stof(str, &pos); // e Eccentricity
                        str = str.substr(pos);
                        datum_GE.Cus = stof(str, &pos); // Cus (radians)
                        str = str.substr(pos);
                        datum_GE.Ahalf = stof(str, &pos); // sqrt(A)

                        // Next Line ORBIT - 3
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.Toe = stof(line, &pos); // Time of Ephemeris (sec of BDT week)
                        str = line.substr(pos);
                        datum_GE.Cic = stof(str, &pos); // Cic (radians)
                        str = str.substr(pos);
                        datum_GE.Omega0 = stof(str, &pos); // OMEGA0 (radians)
                        str = str.substr(pos);
                        datum_GE.Cis = stof(str, &pos); // Cis (radians)

                        // Next Line ORBIT - 4
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.i0 = stof(line, &pos); // i0 (radians)
                        str = line.substr(pos);
                        datum_GE.Crc = stof(str, &pos); // Cic (meters)
                        str = str.substr(pos);
                        datum_GE.w = stof(str, &pos); // omega argument of perigee (radians)
                        str = str.substr(pos);
                        datum_GE.Omegadot = stof(str, &pos); // OMEGA DOT (radians/sec)

                        // Next Line ORBIT - 5
                        std::getline(navFile, line);
                        ++linecount;
                        datum_GE.idot = stof(line, &pos); // IDOT (radians/sec)
                        str = line.substr(pos);
                        datum_GE.week = stof(str, &pos); // codes on L2 (Ignore)
                        str = str.substr(pos);
                        datum_GE.week = stof(str, &pos); // GPS/GAL/BDT week # (to go with Toe)

                        // Now datum_GE is complete push based on prn
                        ephemeris_C[prn - 1].push_back(datum_GE);

                        // Skip remaining lines
                        // Next Line ORBIT - 6
                        std::getline(navFile, line);
                        ++linecount;
                        // Next Line ORBIT - 7
                        std::getline(navFile, line);
                        ++linecount;
                    } catch(std::exception& e) {
                        // cannot process this record, skip lines
                        while(linecount != 7) {
                            std::getline(navFile, line);
                            ++linecount;
                        }
                    }

                } else {
                    // I dont want to process this line
                    continue;
                }
            } // END of FILE
        } else {
            std::cout << "Unable to open navigation file: " << fname << "\n";
            std::cout << "Exiting with non-zero status !\n";
            exit(-1);
        }

        // Closing Navigation File
        navFile.close();
    } // END of for loop over file names
};



void navigation::getPositionR(ephemerisR& initialConditions, int h, triple &pos)
{

	double w10 = initialConditions.px;
	double w20 = initialConditions.py;
	double w30 = initialConditions.pz;
	double w40 = initialConditions.vx;
	double w50 = initialConditions.vy;
	double w60 = initialConditions.vz;
	
	double r = sqrt(w10*w10 + w20*w20 + w30*w30);
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
    
	double k14 = h*((mu_r3 * w10 )+(c20mu *w10)*(w30_2_15)+ w3_2 * w10 + w3t2 * w50 + initialConditions.xdd);
	double k15 = h*((mu_r3 * w20 )+(c20mu *w20)*(w30_2_15)+ w3_2 * w20 - w3t2 * w40 + initialConditions.ydd);
	double k16 = h*((mu_r3 * w30 )+(c20mu *w30)*(3.0 - w30_2_r2_5)+ initialConditions.zdd);
	
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

    
	double k24 = h*((mu_r3 * w10k11)+(c20mu * w10k11  *w30k13_1)+ w3_2 * w10k11 + w3t2 * (w50 + 0.5 * k15) + initialConditions.xdd);
	double k25 = h*((mu_r3 * w20k12)+(c20mu * w20k12  *w30k13_1)+ w3_2 * w20k12 - w3t2 * (w40 + 0.5 * k14) + initialConditions.ydd);
	double k26 = h*((mu_r3 * w30k13)+(c20mu * w30k13  *w30k13_3)+ initialConditions.zdd);


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
    
	double k34 = h*((mu_r3 * w10k21)+(c20mu * w10k21 * w30k23_1)+ w3_2 * w10k21 + w3t2 * (w50 + 0.5 * k25) + initialConditions.xdd);
	double k35 = h*((mu_r3 * w20k22)+(c20mu * w20k22 * w30k23_1)+ w3_2 * w20k22 - w3t2 * (w40 + 0.5 * k24) + initialConditions.ydd);
	double k36 = h*((mu_r3 * w30k23)+(c20mu * w30k23 * w30k23_3)+ initialConditions.zdd);


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
    
	double k44 = h*((mu_r3 * w10k31)+(c20mu * w10k31 * w30k33_1)+ w3_2 * w10k31 + w3t2 * (w50 + k35) + initialConditions.xdd);
	double k45 = h*((mu_r3 * w20k32)+(c20mu * w20k32 * w30k33_1)+ w3_2 * w20k32 - w3t2 * (w40 + k34) + initialConditions.ydd);
	double k46 = h*((mu_r3 * w30k33)+(c20mu * w30k33 * w30k33_3)+ initialConditions.zdd);


//    std::cout << "k41: " << k41 << " k42: " << k42 << " k43: " << k43
//              << " k44: " << k44 << " k45: " << k45 << " k46: " << k46 << "\n";

    double onesixth = 1.0/6.0;
	pos.X = w10 + onesixth*(k11 + 2.0*k21 + 2.0*k31 + k41);
	pos.Y = w20 + onesixth*(k12 + 2.0*k22 + 2.0*k32 + k42);
	pos.Z = w30 + onesixth*(k13 + 2.0*k23 + 2.0*k33 + k43);
//	rxd = w40 + onesixth*(k14 + 2.0*k24 + 2.0*k34 + k44);
//	ryd = w50 + onesixth*(k15 + 2.0*k25 + 2.0*k35 + k45);
//	rzd = w60 + onesixth*(k16 + 2.0*k26 + 2.0*k36 + k46);
    
//  std::cout << rx << "  " << ry << "  " << rz << "\n";

};




void navigation::getPositionGE(ephemerisGE& initial, int t , triple& pos)
{
    // Input t --> is current time in GPS seconds for which postion is to be calculated
    // Output pos --> is the resultant position vector (X,Y,Z) (pos.X,posY,posZ)

    // Compute the time tk from the ephemerides reference epoch Toe (t and Toe are expressed in seconds in the GPS week)
    int tk = t - initial.Toe;

    if(tk > 302400) {
        tk = tk - 604800;
    } else if(tk < -302400) {
        tk = tk + 604800;
    }

    // Compute the mean anomaly Mk for tk
    // Here mu = 3.986005e+14 defined in constants source file
    //     M0 is mean anomaly at refernce epoch (fron nav epoch record)
    //     Ahalf is sqrt of semi-major axis (fron nav epoch record)
    float Mk = initial.M0 + (sqrt(mu) / (initial.Ahalf * initial.Ahalf * initial.Ahalf) + 
                     initial.deltan) * tk;

    // Solve (iteratively) the Kepler equation for the eccentricity anomaly Ek
    // Arguments Mk from previous step
    //          e eccentricity fron nav epoch record
    float Ek = eccAnomaly(Mk, initial.e);

    // Compute the true anomaly vk
    // using Ek computed in previous step and e eccentricity fron nav epoch record
    float vk = atan((sqrt(1 - initial.e * initial.e) * 
                    sin(Ek)) / (cos(Ek) - initial.e));

    // Compute the argument of latitude
    // from: the argument of perigee (w) (fron nav epoch record),
    //       true anomaly vk (computed in previous step),
    //       and corrections Cuc and Cus (fron nav epoch record):
    float uk = initial.w + vk + (initial.Cuc * cos(2 * (initial.w + vk))) + 
               (initial.Cus * sin(2 * (initial.w + vk)));

    // Compute the radial distance rk,
    // considering: corrections Crc and Crs (fron nav epoch record):
    float rk = (initial.Ahalf * initial.Ahalf) * 
               (1 - initial.e * cos(Ek)) + 
               (initial.Crc * cos(2 * (initial.w + vk))) + 
               (initial.Crs * sin(2 * (initial.w + vk)));

    // Compute the inclination of the orbital plane
    // from: the inclination i0 (i0) (fron nav epoch record) at reference time Toe,
    // and corrections Cic and Cis (fron nav epoch record):
    float ik = initial.i0 + (tk * initial.idot) + 
                (initial.Cic * cos(2 * (initial.w + vk))) + 
                (initial.Cis * sin(2 * (initial.w + vk)));

    // Compute the longitude of the ascending node LAMBDAk (Lk) (with respect to Greenwich). This calculation uses:
    // the right ascension at the beginning of the current week (Omega0) (fron nav epoch record),
    // the correction from the apparent sidereal time variation in Greenwich between the beginning of the week and
    // reference time tk,
    // and the change in longitude of the ascending node from the reference time Toe (Toe fron nav epoch record):
    // wE is Earth's rotation rate in radians per sec defined in constants source file.
    // OMEGAdot is rate of node's right ascension (fron nav epoch record)
    float Lk = initial.Omega0 + ((initial.Omegadot - wE) * tk) - 
                (wE * initial.Toe);

    // Compute the coordinates in TRS frame, applying three rotations (around uk, ik and Lk ):
    applyRotations(Lk, ik, uk, rk, pos);
};



// This Routine Calculates eccentricity anomaly Ek
// by Solving (iteratively) the Kepler equation for the eccentricity anomaly, 
// using Newton–Raphson method, Euation -->  Mk = Ek - ( e * Sin(Ek) )
float navigation::eccAnomaly(float M, float e)
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
void navigation::applyRotations(float& Lk, float& ik, float& uk, 
                                float& rk, triple& pos)
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



// This routine has some precision problem !
void navigation::ecefToEllipsoidal(const triple& ecef, triple& ellipsoid)
{
	double phi0 = 0.0;
	double h0 = 0.0;
	double p = sqrt(ecef.X * ecef.X + ecef.Y * ecef.Y);
	double Ni;
	double change_phi;
	double change_h;
    
    //eccentricity squared of elipse cross-section
    double e2 = 1.0 - ( (b_WGS84 * b_WGS84) / (a_WGS84 * a_WGS84) );  

	int iter = 0;
    
    //Calculation of Longitude (East) in degrees
	ellipsoid.Y = (atan(ecef.Y/ecef.X) * toDegrees) + 360.0;  

	//calculate phi(Latitude) by iterative method
	//first set initial values
	phi0 = atan( ecef.Z / ( ( 1.0 - e2 ) * p ) );
	
	do
	{
		iter += 1;
		Ni = a_WGS84 / sqrt( 1.0 - e2 * sin(phi0) * sin(phi0));
		ellipsoid.Z = ( p / cos(phi0) ) - Ni;
		ellipsoid.X = atan( ecef.Z / ( ( 1.0 - e2 * ( Ni / ( Ni + ellipsoid.Z ) ) ) * p ) );
		change_phi = abs( ( (ecef.X - phi0) / ecef.X ) * 100.0 );
		change_h = abs( ( (ellipsoid.Z - h0) / ellipsoid.Z ) * 100.0 );
		phi0 = ecef.X;
		h0 = ellipsoid.Z;
	}
	while(change_phi > 0.0000005 || change_h > 0.0000005);

};




//Routine to calculate satellite elevation
void navigation::satElevAzim(triple& markerECEF, triple& sat, triple& markerEllip, double& elevation, double& azimuth)
{
	//compute line of sight vector
	double p[3];

	double e[3];
	double n[3];
	double u[3];

	double diff[3];
	double mag;

	
	diff[0] = sat.X - markerECEF.X;
	diff[1] = sat.Y - markerECEF.Y;
	diff[2] = sat.Z - markerECEF.Z;

	mag = sqrt( (diff[0] * diff[0]) + (diff[1] * diff[1]) + (diff[2] * diff[2]) );

	p[0] = diff[0] / mag;
	p[1] = diff[1] / mag;
	p[2] = diff[2] / mag;

	e[0] = -1 * sin(markerEllip.X);
	e[1] = cos(markerEllip.X);
	e[2] = 0.0;

	n[0] = -1 * cos(markerEllip.X) * sin(markerEllip.Y);
	n[1] = -1 * sin(markerEllip.X) * sin(markerEllip.Y);
	n[2] = cos(markerEllip.Y);

	u[0] = cos(markerEllip.X) * cos(markerEllip.Y);
	u[1] = sin(markerEllip.X) * cos(markerEllip.Y);
	u[2] = sin(markerEllip.Y);

	elevation =  asin( (p[0] * u[0]) + (p[1] * u[1]) + (p[2] * u[2]) );
	azimuth =  atan( ((p[0] * e[0]) + (p[1] * e[1]) + (p[2] * e[2])) / ((p[0] * n[0]) + (p[1] * n[1]) + (p[2] * n[2])) );
};




//This routine calculates IPP ionospheric pierce point in cartesian
//using sphere-line euation and reference height of ionosphere (given as input)
//marker (station) and satellite positions are also inputs
int navigation::computeIPP(const triple& marker, const triple& sat, const double& rh, triple& IPP, double& coschi)
{
	int status;
	double scaling = 1000000.0;

	//define radius of sphere as radius of earth + reference height of ionosphere
	double r = 6371000.0 + rh * 1000.0 ; //radius in meters, rh given in km
	//scale down
	r = r / scaling;

	//Substituting the equation of the line into the sphere gives a quadratic equation of the form
	// a u2 + b u + c = 0
	// where:
	// a = (x2 - x1)2 + (y2 - y1)2 + (z2 - z1)2
	// b = 2[ (x2 - x1) (x1 - x3) + (y2 - y1) (y1 - y3) + (z2 - z1) (z1 - z3) ]
	// c = x32 + y32 + z32 + x12 + y12 + z12 - 2[x3 x1 + y3 y1 + z3 z1] - r2 
	
	//Defining a,b,c

	double a,b,c;
	
	//scale down coordinates
	double mx = marker.X / scaling;
	double my = marker.Y / scaling;
	double mz = marker.Z / scaling;
	double sx = sat.X / scaling;
	double sy = sat.Y / scaling;
	double sz = sat.Z / scaling;

	a = (sx - mx)*(sx - mx) + (sy - my)*(sy - my) + (sz - mz)*(sz - mz);
	b = 2.0 * ( (sx - mx)*(mx - 0.0) + (sy - my)*(my - 0.0) + (sz - mz)*(mz - 0.0) );
	c = (mx * mx) + (my * my) + (mz * mz) - r*r;
	
	
	//vector for line of sight
	triple los;
	los.X = sx - mx; los.Y = sy - my; los.Z = sz - mz;

	//magnitude of line of sight
	double mag_los = sqrt( (los.X * los.X) + (los.Y * los.Y) + (los.Z * los.Z) );
	
	//Direction Unit vector for los
	triple d_los;
	d_los.X = los.X / mag_los;
	d_los.X = los.Y / mag_los;
	d_los.X = los.Z / mag_los;
	

	//The solutions to this quadratic are described by:
	// ( -b +- sqrt( b*b -4*a*c ) ) / 2*a
	// The exact behaviour is determined by the expression within the square root:
	// b*b -4*a*c
	// If this is less than 0 then the line does not intersect the sphere: return -1
	// If it equals 0 then the line is a tangent to the sphere intersecting it at one point, namely at u = -b/2a,
	// which is not possible as marker is inside sphere so line of sight caannot be a tangent: return -2
	// If it is greater then 0 the line intersects the sphere at two points.

	// Decide exact behaviour
	
	double eb = b*b - 4*a*c;
	double u1 = 0.0;
	double u2 = 0.0;
	triple vec_u1;
	triple vec_u2;
	
	double u1d = 0.0;
	double u2d = 0.0;
	
	double adotb = 0.0;
	double mag_IPP = 0.0;
	
	if(eb < 0.0)
	{
		status = -1;
		return status;
	}
	else if(eb == 0.0)
	{
		status = -2;
		return status;
	}
	else if(eb > 0.0)
	{
		u1 = ( (-1 * b) + sqrt(eb) ) / (2 * a);
		u2 = ( (-1 * b) - sqrt(eb) ) / (2 * a);
		// calculate both points
		vec_u1.X = d_los.X * u1;
		vec_u1.Y = d_los.Y * u1;
		vec_u1.Z = d_los.Z * u1;

		vec_u2.X = d_los.X * u2;
		vec_u2.Y = d_los.Y * u2;
		vec_u2.Z = d_los.Z * u2;
		// to both solutions add marker vector to obtain ipp from origin
		vec_u1.X += mx;
		vec_u1.Y += my;
		vec_u1.Z += mz;

		vec_u2.X += mx;
		vec_u2.Y += my;
		vec_u2.Z += mz;
		
		//Now both vectors start from origin Earth's Center
		// select right solution by selecting shortest distance from solution points to satellite
		
		u1d = sqrt( (sx - vec_u1.X)*(sx - vec_u1.X) + (sy - vec_u1.Y)*(sy - vec_u1.Y) + (sz - vec_u1.Z)*(sz - vec_u1.Z) );
		u2d = sqrt( (sx - vec_u2.X)*(sx - vec_u2.X) + (sy - vec_u2.Y)*(sy - vec_u2.Y) + (sz - vec_u2.Z)*(sz - vec_u2.Z) );

		if(u1d < u2d)
		{
			IPP.X = vec_u1.X;
			IPP.Y = vec_u1.Y;
			IPP.Z = vec_u1.Z;
			status = 0;
		}else
		{
			IPP.X = vec_u2.X;
			IPP.Y = vec_u2.Y;
			IPP.Z = vec_u2.Z;
			status = 0;			
		}
		
			
		//Now calculate coschi between los vector and IPP vector
		//by cos(x) = a.b / |a||b|, where a is los vector and b is IPP vector

		adotb = (los.X * IPP.X) + (los.Y * IPP.Y) + (los.Z * IPP.Z);
		mag_IPP = sqrt( (IPP.X * IPP.X) + (IPP.Y * IPP.Y) + (IPP.Z * IPP.Z) );
		coschi = acos( adotb / (mag_los * mag_IPP) );
		

		IPP.X  *= scaling;
		IPP.Y  *= scaling;
		IPP.Z  *= scaling;

		return status;
	}
};












