/*
    GTEC - Multi Constellation GNSS Derived TEC Calibration Model 
    by T/ICT4D Lab ICTP.
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
*/


#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <exception>
#include "navigation.hpp"
#include "internalTime.hpp"

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
