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



#include <string>
#include <sstream>
#include <exception>
#include <algorithm>
#include "ObsData.hpp"
#include "constants.hpp"

void ObsData::setSysFlags(std::string sysString)
{

    for(int i = 0; i < sysString.size(); ++i)
        {
            if(sysString[i] == 'G')
                readGPS = true;
            else if(sysString[i] == 'R')
                readGLO = true;
            else if(sysString[i] == 'E')
                readGAL = true;
            else if(sysString[i] == 'C')
                readBEI = true;
        }
};

ObsData::ObsData(std::vector<std::string> fvec, std::string sysString)
{
    fnames = fvec;
    hasGPS = hasGLO = hasGAL = hasBEI = false;
    readGPS = readGLO = readGAL = readBEI = false;

    setSysFlags(sysString);

    timeline_main = {};
    if(readGPS)
        {
            GPS_ucTEC = { {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {} };
            for(int i = 0; i < GPS_SIZE; ++i)
                {
                    GPS_Mark[i] = 0;
                }
        }

    if(readGLO)
        {
            GLO_ucTEC = { {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {} };
            for(int i = 0; i < GLO_SIZE; ++i)
                {
                    GLO_Mark[i] = 0;
                }
        }

    if(readGAL)
        {
            GAL_ucTEC = { {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {} };
            for(int i = 0; i < GAL_SIZE; ++i)
                {
                    GAL_Mark[i] = 0;
                }
        }

    if(readBEI)
        {
            BDU_ucTEC = { {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {} };
            for(int i = 0; i < BDU_SIZE; ++i)
                {
                    BDU_Mark[i] = 0;
                }
        }

    for(int i = 0; i < 120; ++i)
        {
            NonZero_Mark[i] = 0;
        }

    numNonZeroArcs = 0;
    
    size_of_S = 0;
};

void ObsData::resetMark()
{
    if(readGPS)
        for(int i = 0; i < GPS_SIZE; ++i)
            {
                GPS_Mark[i] = 0;
            }
    if(readGLO)
        for(int i = 0; i < GPS_SIZE; ++i)
            {
                GLO_Mark[i] = 0;
            }
    if(readGAL)
        for(int i = 0; i < GPS_SIZE; ++i)
            {
                GAL_Mark[i] = 0;
            }
    if(readBEI)
        for(int i = 0; i < GPS_SIZE; ++i)
            {
                BDU_Mark[i] = 0;
            }
};

int ObsData::pad_zero()
{
    if(readGPS && hasGPS)
        {
            for(int i = 0; i < GPS_SIZE; ++i)
                {
                    if(GPS_Mark[i] == 0)
                        {
                            GPS_ucTEC[i].push_back(0);
                        }
                }
        }
    if(readGLO && hasGLO)
        {
            for(int i = 0; i < GLO_SIZE; ++i)
                {
                    if(GLO_Mark[i] == 0)
                        {
                            GLO_ucTEC[i].push_back(0);
                        }
                }
        }
    if(readGAL && hasGAL)
        {
            for(int i = 0; i < GAL_SIZE; ++i)
                {
                    if(GAL_Mark[i] == 0)
                        {
                            GAL_ucTEC[i].push_back(0);
                        }
                }
        }
    if(readBEI && hasBEI)
        {
            for(int i = 0; i < BDU_SIZE; ++i)
                {
                    if(BDU_Mark[i] == 0)
                        {
                            BDU_ucTEC[i].push_back(0);
                        }
                }
        }
    resetMark();
    return 0;
}

void ObsData::read()
{
    for(auto fname : fnames)
        {
            std::ifstream inputFile;
            inputFile.open(fname);

            std::string line;
            std::string str;
            std::string tmp;
            std::size_t pos;
            internalTime epoch_time;
            int lineNumber = 0;

            if(inputFile.is_open())
                {
                    while(std::getline(inputFile, line))
                        {
                            lineNumber += 1;
                            str = line.substr(60);
                            if(str[0] == 'E' && str[1] == 'N' && str[2] == 'D' && str[4] == 'O' && str[5] == 'F' && str[7] == 'H' &&
                               str[8] == 'E' && str[9] == 'A' && str[10] == 'D' && str[11] == 'E' && str[12] == 'R')
                                {
                                    std::cout << "End of Header !!"
                                              << "\n";
                                    break;
                                }

                            if(str[0] == 'R' && str[1] == 'I' && str[2] == 'N' && str[3] == 'E' && str[4] == 'X')
                                version = stof(line);

                            if(str[0] == 'I' && str[1] == 'N' && str[2] == 'T' && str[3] == 'E' && str[4] == 'R' && str[5] == 'V' &&
                               str[6] == 'A' && str[7] == 'L')
                                interval = stoi(line);

                            if(str[0] == 'A' && str[1] == 'P' && str[2] == 'P' && str[3] == 'R' && str[4] == 'O' && str[5] == 'X')
                                {
                                    MarkerPosition.X = stof(line, &pos);
                                    tmp = line.substr(pos);
                                    MarkerPosition.Y = stof(tmp, &pos);
                                    tmp = tmp.substr(pos);
                                    MarkerPosition.Z = stof(tmp);
                                }

                            if(str[0] == 'T' && str[1] == 'I' && str[2] == 'M' && str[3] == 'E' && str[8] == 'F' && str[9] == 'I' &&
                               str[10] == 'R' && str[11] == 'S' && str[12] == 'T')
                                {

                                    TOFO.parse(line, tmp);
                                    TOFO.toUNIXTime();
                                    // Now Get system in which this time is represented
                                    pos = 0;
                                    while(isspace(tmp[pos]))
                                        {
                                            pos += 1;
                                        }
                                    TOFO_system = tmp.substr(pos, 3);
                                    hasTOFO = true;
                                }

                            if(line[0] == 'G' && str[0] == 'S' && str[1] == 'Y' && str[2] == 'S' && str[10] == 'O' &&
                               str[11] == 'B' && str[12] == 'S')
                                hasGPS = true;
                            if(line[0] == 'R' && str[0] == 'S' && str[1] == 'Y' && str[2] == 'S' && str[10] == 'O' &&
                               str[11] == 'B' && str[12] == 'S')
                                hasGLO = true;
                            if(line[0] == 'E' && str[0] == 'S' && str[1] == 'Y' && str[2] == 'S' && str[10] == 'O' &&
                               str[11] == 'B' && str[12] == 'S')
                                hasGAL = true;
                            if(line[0] == 'C' && str[0] == 'S' && str[1] == 'Y' && str[2] == 'S' && str[10] == 'O' &&
                               str[11] == 'B' && str[12] == 'S')
                                hasBEI = true;
                        }
                }
            else
                {
                    std::cout << "Unable to open observation file: " << fname << "\n";
                    std::cout << "Exiting with non-zero status !\n";
                    exit(-1);
                }

            // Now read data portion
            int epoch_counter = 0;
            float C1, C2, L1, L2;
            int lli;
            int SatID;

            float GPS_f12 = (1.0 / (GPS_f2 * GPS_f2)) - (1.0 / (GPS_f1 * GPS_f1));
            float GLO_f12 = (1.0 / (GLO_f2 * GLO_f2)) - (1.0 / (GLO_f1 * GLO_f1));
            float GAL_f12 = (1.0 / (GAL_f2 * GAL_f2)) - (1.0 / (GAL_f1 * GAL_f1));
            float BDU_f12 = (1.0 / (BDU_f2 * BDU_f2)) - (1.0 / (BDU_f1 * BDU_f1));

            float GPS_tau = 1.0 / (40.3 * GPS_f12);
            float GLO_tau = 1.0 / (40.3 * GLO_f12);
            float GAL_tau = 1.0 / (40.3 * GAL_f12);
            float BDU_tau = 1.0 / (40.3 * BDU_f12);

            while(!inputFile.eof())
                {
                    std::getline(inputFile, line);
                    lineNumber += 1;

                    // check for blank lines
                    if(line.size() == 0)
                        {
                            continue;
                        }

                    if(line[0] == '>')
                        {
                            // Epoch started, fill zeros for previous epoch
                            if(epoch_counter != 0)
                                {
                                    pad_zero();
                                }
                            // Now increment Epoch Counter
                            epoch_counter += 1;

                            // read convert and push time in timeline
                            // std::cout << line << "\t";
                            epoch_time.parse(line);
                            epoch_time.toUNIXTime();
                            timeline_main.push_back(epoch_time.UNIX);
                        }
                    else
                        {
                            if(line[0] == 'G' && readGPS)
                                {
                                    // Processing GPS Line...
                                    if(line.size() < 85)
                                        {
                                            continue;
                                        }

                                    tmp = line.substr(1);
                                    SatID = stoi(tmp, &pos);
                                    tmp = tmp.substr(pos);

                                    // Asuming sequence C1 lli L1 lli S1 C2 lli L2 lli S2 ........
                                    // read in sequence the values L1 and L2
                                    try
                                        {
                                            C1 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            lli = stoi(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            L1 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            lli = stoi(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            C2 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            C2 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            lli = stoi(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            L2 = stof(tmp, &pos);
                                        }
                                    catch(std::exception& e)
                                        {
                                            std::cout << "Exception occured..  " << e.what() << "\n";
                                            std::cout << "Error parsing file: " << fname << "  at line: " << lineNumber << "\n";
                                            std::cout << "Skipping the line\n";
                                            continue;
                                        }

                                    // Now Calculate uncalibrated TEC from Phase
                                    GPS_ucTEC[SatID - 1].push_back(GPS_tau * c * ((L1 * Glmda1) - (L2 * Glmda2)) / TECU);
                                    GPS_Mark[SatID - 1] = 1;
                                    markNonZeroArcs(CONSTELLATION_ID_GPS, SatID);
                                }
                            else if(line[0] == 'R' && readGLO)
                                {
                                    // Processing GLONASS Line...
                                    if(line.size() < 85)
                                        {
                                            continue;
                                        }

                                    tmp = line.substr(1);
                                    SatID = stoi(tmp, &pos);
                                    tmp = tmp.substr(pos);

                                    // Asuming sequence C1 lli L1 lli S1 C2 lli L2 lli S2 ........
                                    // read in sequence the values L1 and L2
                                    try
                                        {
                                            C1 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            lli = stoi(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            L1 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            lli = stoi(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            C2 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            C2 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            lli = stoi(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            L2 = stof(tmp, &pos);
                                        }
                                    catch(std::exception& e)
                                        {
                                            std::cout << "Exception occured..  " << e.what() << "\n";
                                            std::cout << "Error parsing file: " << fname << "  at line: " << lineNumber << "\n";
                                            std::cout << "Skipping the line\n";
                                            continue;
                                        }

                                    // Now Calculate uncalibrated TEC from Phase
                                    GLO_ucTEC[SatID - 1].push_back(GLO_tau * c * ((L1 * Rlmda1) - (L2 * Rlmda2)) / TECU);
                                    GLO_Mark[SatID - 1] = 1;
                                    markNonZeroArcs(CONSTELLATION_ID_GLO, SatID);
                                }
                            else if(line[0] == 'E' && readGAL)
                                {
                                    // Processing GALILEO Line...
                                    if(line.size() < 85)
                                        {
                                            continue;
                                        }

                                    tmp = line.substr(1);
                                    SatID = stoi(tmp, &pos);
                                    tmp = tmp.substr(pos);

                                    // Asuming sequence C1 lli L1 lli S1 C2 lli L2 lli S2 ........
                                    // read in sequence the values L1 and L2
                                    try
                                        {
                                            C1 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            lli = stoi(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            L1 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            lli = stoi(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            C2 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            C2 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            lli = stoi(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            L2 = stof(tmp, &pos);
                                        }
                                    catch(std::exception& e)
                                        {
                                            std::cout << "Exception occured..  " << e.what() << "\n";
                                            std::cout << "Error parsing file: " << fname << "  at line: " << lineNumber << "\n";
                                            std::cout << "Skipping the line\n";
                                            continue;
                                        }

                                    // Now Calculate uncalibrated TEC from Phase
                                    GAL_ucTEC[SatID - 1].push_back(GAL_tau * c * ((L1 * Elmda1) - (L2 * Elmda2)) / TECU);
                                    GAL_Mark[SatID - 1] = 1;
                                    markNonZeroArcs(CONSTELLATION_ID_GAL, SatID);
                                }
                            else if(line[0] == 'C' && readBEI)
                                {
                                    // Processing BEIDOU Line...
                                    if(line.size() < 85)
                                        {
                                            continue;
                                        }

                                    tmp = line.substr(1);
                                    SatID = stoi(tmp, &pos);
                                    tmp = tmp.substr(pos);

                                    // Asuming sequence C1 lli L1 lli S1 C2 lli L2 lli S2 ........
                                    // read in sequence the values L1 and L2
                                    try
                                        {
                                            C1 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            lli = stoi(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            L1 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            lli = stoi(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            C2 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            C2 = stof(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            lli = stoi(tmp, &pos);
                                            tmp = tmp.substr(pos);

                                            L2 = stof(tmp, &pos);
                                        }
                                    catch(std::exception& e)
                                        {
                                            std::cout << "Exception occured..  " << e.what() << "\n";
                                            std::cout << "Error parsing file: " << fname << "  at line: " << lineNumber << "\n";
                                            std::cout << "Skipping the line\n";
                                            continue;
                                        }

                                    // Now Calculate uncalibrated TEC from Phase
                                    BDU_ucTEC[SatID - 1].push_back(BDU_tau * c * ((L1 * Blmda1) - (L2 * Blmda2)) / TECU);
                                    BDU_Mark[SatID - 1] = 1;
                                    markNonZeroArcs(CONSTELLATION_ID_BDU, SatID);
                                }
                        }
                }
            // Now pad zero for the last epoch
            pad_zero();

            // Closing Input file
            inputFile.close();
        }

    // Now set Non zero arc markings
    getnumNonZeroArcs();
};

int ObsData::dumpArc(char sys, int prn)
{
    switch(sys)
        {
        case 'G':

            for(auto ucTec : GPS_ucTEC[prn - 1])
                {
                    std::cout << ucTec << "\n";
                }
            break;

        case 'R':

            for(auto ucTec : GLO_ucTEC[prn - 1])
                {
                    std::cout << ucTec << "\n";
                }
            break;

        case 'E':

            for(auto ucTec : GAL_ucTEC[prn - 1])
                {
                    std::cout << ucTec << "\n";
                }
            break;

        case 'B':

            for(auto ucTec : BDU_ucTEC[prn - 1])
                {
                    std::cout << ucTec << "\n";
                }
            break;
        default:
            return -1;
        }
    return 0;
};

int ObsData::dumpArcByTime(char sys, int prn)
{
    int sz = timeline_main.size();
    switch(sys)
        {
        case 'G':
            if(GPS_ucTEC[prn - 1].size() != sz)
                {
                    std::cout << "Arc G" << prn << " size not equals main timeline size (" << GPS_ucTEC[prn - 1].size() << " vs" << timeline_main.size() << ").\n";
                    exit(-1);
                }
            for(int i = 0; i < sz; ++i)
                {
                    std::cout << timeline_main[i] << "  " << GPS_ucTEC[prn - 1][i] << "\n";
                }
            break;

        case 'R':
            if(GLO_ucTEC[prn - 1].size() != sz)
                {
                    std::cout << "Arc R" << prn << " size not equals main timeline size (" << GLO_ucTEC[prn - 1].size() << " vs" << timeline_main.size() << ").\n";
                    exit(-1);
                }
            for(int i = 0; i < sz; ++i)
                {
                    std::cout << timeline_main[i] << "  " << GLO_ucTEC[prn - 1][i] << "\n";
                }
            break;

        case 'E':
            if(GAL_ucTEC[prn - 1].size() != sz)
                {
                    std::cout << "Arc E" << prn << " size not equals main timeline size (" << GAL_ucTEC[prn - 1].size() << " vs" << timeline_main.size() << ").\n";
                    exit(-1);
                }
            for(int i = 0; i < sz; ++i)
                {
                    std::cout << timeline_main[i] << "  " << GAL_ucTEC[prn - 1][i] << "\n";
                }
            break;

        case 'B':
            if(BDU_ucTEC[prn - 1].size() != sz)
                {
                    std::cout << "Arc C" << prn << " size not equals main timeline size (" << BDU_ucTEC[prn - 1].size() << " vs" << timeline_main.size() << ").\n";
                    exit(-1);
                }
            for(int i = 0; i < sz; ++i)
                {
                    std::cout << timeline_main[i] << "  " << BDU_ucTEC[prn - 1][i] << "\n";
                }
        default:
            return -1;
        }
    return 0;
};

int ObsData::dumpArcBinary(char sys, int prn)
{
    switch(sys)
        {
        case 'G':
            for(auto val : GPS_ucTEC[prn - 1])
                {
                    if(val == 0.0)
                        std::cout << 0;
                    else
                        std::cout << 1;
                }
            std::cout << "\n";

        case 'R':
            for(auto val : GLO_ucTEC[prn - 1])
                {
                    if(val == 0.0)
                        std::cout << 0;
                    else
                        std::cout << 1;
                }
            std::cout << "\n";

        case 'E':
            for(auto val : GAL_ucTEC[prn - 1])
                {
                    if(val == 0.0)
                        std::cout << 0;
                    else
                        std::cout << 1;
                }
            std::cout << "\n";

        case 'C':
            for(auto val : BDU_ucTEC[prn - 1])
                {
                    if(val == 0.0)
                        std::cout << 0;
                    else
                        std::cout << 1;
                }
            std::cout << "\n";

        default:
            return -1;
        }
    return 0;
};


//This Function dumps raw matrix to given standard output
//Useful for debugging purposes
void ObsData::dumpRawMatrix(const double* mat, int& dim1, int& dim2)
{
    for (int i=0; i<dim1; ++i)
    {
        for (int j=0; j<dim2; ++j)
        {
            std::cout << mat[i*dim2 + j] << " ";
        }
        std::cout << "\n";
    }
};



int ObsData::dumpSizes()
{
    int id = 1;
    //This routine prints internal data structure sizes
    //print timeline size
    std::cout << "Time Line Size: " << timeline_main.size() << "\n";
    if(readGPS && hasGPS)
        {
            for(auto prn : GPS_ucTEC)
                {
                    std::cout << "GPS " << id << "  " << prn.size() << "\n";
                    id += 1;
                }
            id = 1;
        }

    if(readGLO && hasGLO)
        {
            for(auto prn : GLO_ucTEC)
                {
                    std::cout << "GLONASS " << id << "  " << prn.size() << "\n";
                    id += 1;
                }
            id = 1;
        }

    if(readGAL && hasGAL)
        {
            for(auto prn : GAL_ucTEC)
                {
                    std::cout << "GALILEO " << id << "  " << prn.size() << "\n";
                    id += 1;
                }
            id = 1;
        }

    if(readBEI && hasBEI)
        {
            for(auto prn : BDU_ucTEC)
                {
                    std::cout << "BEIDOU " << id << "  " << prn.size() << "\n";
                    id += 1;
                }
            id = 1;
        }
    return 0;
};

void ObsData::markNonZeroArcs(int constellation, int prn)
{
    //This routine marks the arcs which are having some data
    //usefull before pre-processing,
    //constellation = 1 = GPS
    //constellation = 2 = GLONASS
    //constellation = 3 = GALILEO
    //constellation = 4 = BEIDOU
    int index;

    if(constellation == 1)
        index = prn - 1;
    else if(constellation == 2)
        index = 32 + prn - 1;
    else if(constellation == 3)
        index = 56 + prn - 1;
    else if(constellation == 4)
        index = 86 + prn - 1;
    else
        {
            std::cout << "Invalid Constellation ID in markNonZeroArcs(). Exiting with status -1.\n";
            exit(-1);
        }
    NonZero_Mark[index] = 1;
}

void ObsData::setArcStartEnd()
{
    float* s = NULL;
    float* e = NULL;
    //This routine sets the pointer pair(start,end) vetor of arcs
    //assuming reduced data set, from begining and end as per
    //Number of hours to reject variable
    int reject = (12 * 60 * 60) / interval;
    
    for(int i = 0; i < 120; ++i)
        {
            //check in mark array
            if(NonZero_Mark[i] == 1)
                {
                    if(i < 32)
                        {
                            //GPS Arc
                            s = reinterpret_cast<float*>(&*GPS_ucTEC[i].begin()+reject);
                            e = reinterpret_cast<float*>(&*GPS_ucTEC[i].end()-reject);
                            arcs.push_back(ptr_pair(s, e));
                            //Aprn.push_back(i);
                            //ASP.push_back(s);
                        }
                    else if(i < 56)
                        {
                            //GLONASS Arc
                            s = reinterpret_cast<float*>(&*(GLO_ucTEC[i - 32].begin()+reject));
                            e = reinterpret_cast<float*>(&*(GLO_ucTEC[i - 32].end()-reject));
                            arcs.push_back(ptr_pair(s, e));
                            //Aprn.push_back(i);
                            //ASP.push_back(s);
                        }
                    else if(i < 86)
                        {
                            //GALILEO Arc
                            s = reinterpret_cast<float*>(&*(GAL_ucTEC[i - 56].begin()+reject));
                            e = reinterpret_cast<float*>(&*(GAL_ucTEC[i - 56].end()-reject));
                            arcs.push_back(ptr_pair(s, e));
                            //Aprn.push_back(i);
                            //ASP.push_back(s);
                        }
                    else if(i < 120)
                        {
                            //BEIDOU Arc
                            s = reinterpret_cast<float*>(&*(BDU_ucTEC[i - 86].begin()+reject));
                            e = reinterpret_cast<float*>(&*(BDU_ucTEC[i - 86].end()-reject));
                            arcs.push_back(ptr_pair(s, e));
                            //Aprn.push_back(i);
                            //ASP.push_back(s);
                        }
                }
        }
};

void ObsData::pre_process(int minArcLen, int intrpolIntrvl, int deg)
{
    //This routine perform pre processing on internal data structure carrying arcs
    //Input: minArcLen = minimum arc length (minutes) to be considered valid for calibration
    //Input: intrpolIntrvl = Maximum interval for which interpolatio will be performed
    // The routine assumes that a flag array exist which describs which prns are actually having any data.
    // Therefore it does not rely on read/has flags which describe whether data for a particular constellation
    // was found in the file or was requested to be processed, and only use the the flag array to define
    // the needed arcs to be pre-processed.

    //setting start and end pointers for each arc (initially to the whole arc length)
    //this would serve as input to other steps in pre-processing to modify arcs
    setArcStartEnd();

    //pre processing step 1:
    //cut arcs when corresponding satellite elevation is out of minElevation and maxElevation
    //to be implemented by means of processing navigation files

    //pre-processing step 2 :
    //cut arcs when there is no contiguous data for 5 mins (300 seconds)
    //This means either satellite went out of sight or a long gap
    float* s = NULL;
    float* e = NULL;
    float* p = NULL;
    float* lastNonZero = NULL;

    for(auto arc : arcs)
        {
            //Move the start
            s = arc.start;
            while(*s == 0.0)
                {
                    s += 1;
                }
            //Move the end
            e = arc.end;
            while(*(e - 1) == 0.0)
                {
                    e -= 1;
                }
            arcs2.push_back(ptr_pair(s, e));
        }



    int gap = 0;
    bool arcNotBroken = true;
    int arc2_counter = 0;
    for(auto arc : arcs2)
        {
            arcNotBroken = true;
            s = p = arc.start;
            while(p != arc.end)
                {
                    //check if there is a gap
                    if(*p == 0.0)
                        {
                            gap += interval;
                        }
                    else
                        {
                            gap = 0;
                            //set address of last non zero value
                            lastNonZero = p;
                        }

                    if(gap > intrpolIntrvl)
                        {
                            arcNotBroken = false;
                            while(*p == 0.0)
                                {
                                    ++p;
                                }
                            e = lastNonZero + 1;
                            if(e - s >= (minArcLen * 60 / interval))
                                {arcs3.push_back(ptr_pair(s, e));
                                //ASP3.push_back(ASP[arc2_counter]);
                                //Aprn3.push_back(Aprn[arc2_counter]);
                                }
                            gap = 0;
                            s = p;
                            lastNonZero = p;
                        }
                    ++p;
                }
            if(arcNotBroken && (e - s) >= (minArcLen * 60 / interval))
                {
                    arcs3.push_back(arc);
                    //ASP3.push_back(ASP[arc2_counter]);
                    //Aprn3.push_back(Aprn[arc2_counter]);
                }
                
            arc2_counter += 1;
            
        }

    //pre-processing step 3 :
    //Interpolate missing values
    for(auto arc : arcs3)
        {
            size_of_S += arc.end - arc.start; //count all total values 
            p = arc.start;
            while(p != arc.end)
                {
                    if(*p == 0.0)
                        {
                            //Do interpolation
                            lagrangeInterpolation(p, arc.start, arc.end, deg);
                        }
                    ++p;
                }
        }

    //pre-processing step 4 :
    //calculate first differences, quartiles, and level out-liers

    std::vector<float> FDiff;
    std::vector<float> FDiff_sorted;

    //variables for quartiles
    float Q1, Q2, Q3;
    int vec_size;
    //Inter Quartile Range
    float IQR;
    float lowerBound;
    float upperBound;
    int t2minust1 = float(interval);
    float sum;

    for(auto arc : arcs3)
        {
            p = arc.start + 1;
            while(p != arc.end)
                {
                    //there is nothing in division like t2 - t1
                    //because we have values each interval
                    //which means t2 - t1 will be fixed (interval) across arc
                    FDiff.push_back((*p - *(p - 1)) / t2minust1);
                    ++p;
                }

            //Now sort the FDiff for median and quartiles
            FDiff_sorted = FDiff;
            std::sort(FDiff_sorted.begin(), FDiff_sorted.end());
            vec_size = FDiff_sorted.size();
            if(vec_size % 2 == 0)
                {
                    //Even
                    Q2 = (FDiff_sorted[(vec_size / 2) - 1] + FDiff_sorted[vec_size / 2]) / 2.0;
                    if((vec_size / 2) % 2 == 0)
                        {
                            //Even
                            Q1 = (FDiff_sorted[(vec_size / 4) - 1] + FDiff_sorted[vec_size / 4]) / 2.0;
                            Q3 = (FDiff_sorted[(vec_size / 4) * 3 - 1] + FDiff_sorted[(vec_size / 4) * 3]) / 2.0;
                        }
                    else
                        {
                            //Odd
                            Q1 = FDiff_sorted[(vec_size / 2 - 1) / 2];
                            Q3 = FDiff_sorted[(vec_size / 2) + (vec_size / 2 - 1) / 2];
                        }
                }
            else
                {
                    //Odd
                    Q2 = FDiff_sorted[(vec_size - 1) / 2];
                    if(((vec_size - 1) / 2) % 2 == 0)
                        {
                            //Even
                            Q1 = (FDiff_sorted[((vec_size - 1) / 4) - 1] + FDiff_sorted[(vec_size - 1) / 4]) / 2.0;
                            Q3 = (FDiff_sorted[((vec_size - 1) / 4) * 3] + FDiff_sorted[((vec_size - 1) / 4) * 3 + 1]) / 2.0;
                        }
                    else
                        {
                            //Odd
                            Q1 = FDiff_sorted[((vec_size - 1) / 2 - 1) / 2];
                            Q3 = FDiff_sorted[(vec_size - 1) / 2 + ((vec_size - 1) / 2 + 1) / 2];
                        }
                }

            //Now calculate IQR
            IQR = Q3 - Q1;
            lowerBound = Q1 - 1.5 * IQR;
            upperBound = Q3 + 1.5 * IQR;

            //Now go over the arc and remove out-liers
            //by leveling them to partial mean around out-lier
            //used vector here would be actual un-sorted FDiff
            //so that we can track back the out-lier which caused that jump
            p = arc.start + 1;
            sum = 0.0;
            for(auto val : FDiff)
                {
                    if(val < lowerBound || val > upperBound)
                        {
                            //p is the pointer to value that caused val as outlier
                            if(((p - 3) >= arc.start) && ((p + 3) <= (arc.end - 1)))
                                {
                                    //get 3 prev points and 3 next points
                                    sum = sum + *(p - 3) + *(p - 2) + *(p - 1);
                                    sum = sum + *(p + 3) + *(p + 2) + *(p + 1);
                                }
                            else if(p - 3 < arc.start)
                                {
                                    //take 6 next points
                                    sum = sum + *(p + 6) + *(p + 5) + *(p + 4) + *(p + 3) + *(p + 2) + *(p + 1);
                                }
                            else
                                {
                                    //take 6 prev points
                                    sum = sum + *(p - 6) + *(p - 5) + *(p - 4) + *(p - 3) + *(p - 2) + *(p - 1);
                                }

                            //nnow set the out-lier equal to mean
                            *p = sum / 6.0;
                            sum = 0.0;
                        }
                    ++p;
                }

            //clear vector
            FDiff.clear();
        }

    //Pre-Processing is now complete
};

void ObsData::getnumNonZeroArcs()
{
    for(int i = 0; i < 120; ++i)
        {
            if(NonZero_Mark[i] == 1)
                numNonZeroArcs += 1;
        }
};

void ObsData::dumpNonZeroArcs()
{
    //This routine prints non zero arcs flags array to identify which arcs have no data at all.
    for(int i = 0; i < 120; ++i)
        {
            if(i < 32)
                std::cout << "GPS " << i + 1 << " " << NonZero_Mark[i] << "\n";
            else if(i < 56)
                std::cout << "GLO " << i + 1 - 32 << " " << NonZero_Mark[i] << "\n";
            else if(i < 86)
                std::cout << "GAL " << i + 1 - 56 << " " << NonZero_Mark[i] << "\n";
            else
                std::cout << "BDU " << i + 1 - 86 << " " << NonZero_Mark[i] << "\n";
        }
    std::cout << "Number of Total Non Zero Arcs:  " << numNonZeroArcs << "\n";
};

int ObsData::lagrangeInterpolation(float* target, float* s, float* e, int deg)
{
    if(deg % 2 != 0)
        return -1;
    //Traverse backwards from target
    //untill:
    //       you get int(deg/2) non-zero values
    //       or you reach arc start (s) in which case also set direction flag
    //       append numbers in vector xi
    int half = int(deg / 2);
    float* t = target - 1;
    std::vector<float> xi = { 0.0 }; //xi pints for interpolation (ignore index 0)
    std::vector<int> ti = { 0 };     //timeline for xi points (ignore index 0)
    float* lastUp = NULL;
    float* lastDown = NULL;

    int time = 0;
    bool upflag = false;
    while(true)
        {
            time -= interval;
            if(xi.size() - 1 == half)
                break;
            if(t == (s - 1))
                {
                    upflag = true;
                    break;
                }

            if(*t != 0.0)
                {
                    xi.insert(xi.begin(), *t);
                    ti.insert(ti.begin(), time);
                    lastUp = t;
                    --t;
                }
            else
                {
                    --t;
                    continue;
                }
        }

    //Traverse forwards from target
    //untill:
    //       you get deg - xi.size() non-zero values
    //       or you reach arc end (e) in which case also set direction flag
    //       append numbers in vector xi
    bool downflag = false;
    t = target + 1;
    time = 0;
    while(true)
        {
            time += interval;
            if(xi.size() - 1 == deg)
                break;
            if(t == e)
                {
                    downflag = true;
                    break;
                }

            if(*t != 0.0)
                {
                    xi.push_back(*t);
                    ti.push_back(time);
                    lastDown = t;
                    ++t;
                }
            else
                {
                    ++t;
                    continue;
                }
        }

    //check if xi.size() == deg
    //    YES ---> go for interpolation
    //    NO:
    //       get more points from direction for which flag not set
    //       if both flags set, return -1 (not enough points)
    if(xi.size() - 1 < deg)
        {
            if(upflag && downflag)
                {
                    return -2;
                }
            else if(upflag)
                {
                    //go downward
                    t = lastDown + 1;
                    time = ti.back();
                    while(true)
                        {
                            time += interval;
                            if(xi.size() - 1 == deg)
                                break;
                            if(t == e)
                                return -3;

                            if(*t != 0.0)
                                {
                                    xi.push_back(*t);
                                    ti.push_back(time);
                                    ++t;
                                }
                        }
                }
            else
                {
                    //go upward
                    t = lastUp - 1;
                    time = *ti.begin();
                    while(true)
                        {
                            time -= interval;
                            if(xi.size() - 1 == deg)
                                break;
                            if(t == (s - 1))
                                return -4;
                            if(*t != 0.0)
                                {
                                    xi.insert(xi.begin(), *t);
                                    ti.insert(ti.begin(), time);
                                    --t;
                                }
                        }
                }
        }

    //Now we have enough points for interpolation
    float product1 = 1.0;
    float product2 = 1.0;
    float sum = 0.0;
    for(int i = 0; i < ti.size(); ++i)
        {
            if(ti[i] == 0)
                continue;

            for(int j = 0; j < ti.size(); ++j)
                {
                    if(ti[j] == 0 || i == j)
                        continue;

                    product1 = product1 * (0.0 - ti[j]);
                    product2 = product2 * (ti[i] - ti[j]);
                }
            sum = sum + (xi[i] * product1 / product2);
            product1 = product2 = 1.0;
        }
    *target = sum;
    return 0;
};

int ObsData::dumpArcBinaryPtrsAll()
{
    float* s = NULL;
    float* e = NULL;
    int arc_count = 0;

    for(auto arc : arcs3)
        {
            s = arc.start;
            e = arc.end;
            arc_count += 1;
            std::cout << "Arc # " << arc_count << "\n";

            while(s != e)
                {
                    if(*s == 0.0)
                        std::cout << 0;
                    else
                        std::cout << 1;
                    ++s;
                }
            std::cout << "\n";
        }
    return 0;
};

int ObsData::dumpArcValuePtrsAll()
{
    int arc_count = 0;
    float* p = NULL;

    for(auto arc : arcs3)
        {
            arc_count += 1;
            std::cout << "Arc # " << arc_count << "\n";
            p = arc.start;
            while(p != arc.end)
                {
                    std::cout << *p << ", ";
                    ++p;
                }
            std::cout << "\n\n";
        }
    return 0;
};







