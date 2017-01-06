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
