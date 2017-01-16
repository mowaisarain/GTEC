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

#include "igrf.hpp"
#include <fstream>
#include <iostream>

std::vector<std::string> igrf::linesplit(std::string str)
{
    std::string field = "";
    char tmp;
    size_t fillcount = 1;
    std::vector<std::string> str_vec;
    for(int i = 0; i < str.length(); ++i)
        {
            tmp = str[i];
            if(tmp == ' ' || tmp == '\t')
                {
                    if(field.length() > 0)
                        {
                            str_vec.push_back(field);
                            field.clear();
                        }
                }
            else
                {
                    field.append(fillcount, tmp);
                }
        }
    str_vec.push_back(field);
    return str_vec;
};

igrf::igrf(std::string fname)
{
    //Allocate memory
    double* pnm = new double[pnmDim];
    double* gnm = new double[nDim * mDim * tDim];
    double* hnm = new double[nDim * mDim * tDim];
    double* svg = new double[nDim * mDim];
    double* svh = new double[nDim * mDim];
    
    
    
    arraySize = numTimeCols * colSize;
    igrfCoeffs = (double*) calloc(arraySize,sizeof(double));
    
    
    
    
    
    std::ifstream igrf_file;
    igrf_file.open(fname);
    std::string line;
    int linecount = 0;
    std::vector<std::string> fields;
    std::vector<std::string> years;

    int n, m, t;
    
    int offSet = 0;

    if(igrf_file.is_open())
        {
            while(getline(igrf_file, line))
                {
                    linecount += 1;
                    fields = linesplit(line);

                    if(linecount > 4)
                        {
                            n = stoi(fields[1]);
                            m = stoi(fields[2]);
                            if(fields[0] == "g")
                                {
                                    for(std::pair<int, std::vector<std::string>::iterator>
                                            i(0, fields.begin() + 3);
                                        i.first < colSize && i.second != fields.end() - 1;
                                        ++i.first, ++i.second)
                                        {
					    //calculate offset
					    
                                            t = std::stoi(*i.first) - 1900;
                                            gnm[n * mDim * tDim + m * tDim + t] = std::stof(*i.second);
                                        }
                                    svg[n * mDim + m] = std::stof(fields.back());
                                }
                            else if(fields[0] == "h")
                                {
                                    for(std::pair<std::vector<std::string>::iterator, std::vector<std::string>::iterator>
                                            i(years.begin() + 3, fields.begin() + 3);
                                        i.first != years.end() - 1 && i.second != fields.end() - 1;
                                        ++i.first, ++i.second)
                                        {
                                            t = std::stoi(*i.first) - 1900;
                                            hnm[n * mDim * tDim + m * tDim + t] = std::stof(*i.second);
                                        }
                                    svh[n * mDim + m] = std::stof(fields.back());
                                }
                        }
                    else if(linecount == 4)
                        {
                            for(std::vector<std::string>::iterator it = fields.begin(); it != fields.end(); ++it)
                                years.push_back(*it);
                        }
                }
            igrf_file.close();
        }
    else
        {
            std::cout << "Unable to open igrf file\n";
            exit(-1);
        }

    //check input
    int status = checkInput();
    if(status != 0)
        {
            std::cout << "Invalid igrf file..\n"; 
            std::cout << "Exiting with non zero exit status.."; 
            exit(-1);
        }
};



int igrf::checkInput()
{
    double hnm331900 = 523;
    double gnm321955 = 1288;
    double svg43 = 4.1;
    double svh53 = -1.2;
    //double one = hnm[3][3][0];
    double one = hnm[3 * mDim * tDim + 3 * tDim + 0];

    //double two = gnm[3][2][55];
    double two = gnm[3 * mDim * tDim + 2 * tDim + 55];

    //double three = svg[4][3];
    double three = svg[4 * mDim + 3];

    //double four = svh[5][3];
    double four = svh[5 * mDim + 3];

    if((hnm331900 == one) && (gnm321955 == two) && (svg43 == three) && (svh53 == four))
        {
            //std::cout << "SUCCESS\n";
            return 0;
        }
    else
        {
            //std::cout << hnm331900 << " " << one << "\n";
            //std::cout << gnm321955 << " " << two << "\n";
            //std::cout << svg43 << " " << three << "\n";
            //std::cout << svh53 << " " << four << "\n";
            //std::cout << "FAIL\n";
            return -1;
        }
};

void igrf::getGaussCoeff(const int& t,
                        const int& n,
                        const int& m,
                        double& gnmt,
                        double& hnmt)
{

    if(t < 1900 || t > 2020)
        {
            std::cout << "Invalid year for IGRF12\n";
            exit(-1);
        }

    int ti = t - 1900;
    int T0 = ti - (ti % 5);
    int T05 = T0 + 5;

    if(ti == 115)
        {
            gnmt = gnm[n * mDim * tDim + m * tDim + ti];
            hnmt = hnm[n * mDim * tDim + m * tDim + ti];
        }
    else if(ti > 115)
        {
            gnmt = svg[n * mDim + m];
            hnmt = svh[n * mDim + m];
        }
    else
        {
            gnmt = (gnm[n * mDim * tDim + m * tDim + T05] - gnm[n * mDim * tDim + m * tDim + T0]) / 5;
            hnmt = (hnm[n * mDim * tDim + m * tDim + T05] - hnm[n * mDim * tDim + m * tDim + T0]) / 5;
        }
};

void igrf::computePnm(double& theta)
{
    //Reference: http://ciks.cbt.nist.gov/~garbocz/paper134/node10.html

    double x = std::cos(theta);
    double s = std::sqrt(1.0 - (x * x));

    double x2 = x * x;
    double x3 = x2 * x;
    double x4 = x3 * x;
    double x5 = x4 * x;
    double x6 = x5 * x;
    double x8 = x6 * x * x;

    double s2 = s * s;
    double s3 = s2 * s;
    double s4 = s3 * s;
    double s5 = s4 * s;
    double s6 = s5 * s;
    double s7 = s6 * s;
    double s8 = s6 * s;

    pnm[0] = 1.0;

    pnm[9] = x;
    pnm[10] = s;

    pnm[18] = 0.5 * ((3.0 * x2) - 1);
    pnm[19] = 3.0 * x * s;
    pnm[20] = 3.0 * (1.0 - x2);

    pnm[27] = 0.5 * x * ((5.0 * x2) - 3.0);
    pnm[28] = 1.5 * ((5.0 * x2) - 1.0) * s;
    pnm[29] = 15.0 * x * (1.0 - x2);
    pnm[30] = 15.0 * s3;

    pnm[36] = 0.125 * ((35.0 * x4) - (30.0 * x2) + 3.0);
    pnm[37] = 2.5 * ((7.0 * x3) - (3.0 * x));
    pnm[38] = 7.5 * (7.0 * x2 - 1) * (1.0 - x2);
    pnm[39] = 105.0 * x * s3;
    pnm[40] = 105.0 * s4;

    pnm[45] = 0.125 * x * ((63.0 * x4) - (70.0 * x2) + 15.0);
    pnm[46] = 1.875 * s * ((21.0 * x4) - (14.0 * x2) + 1.0);
    pnm[47] = 52.5 * x * (1.0 - x2) * (3.0 * x2 - 1.0);
    pnm[48] = 52.5 * s3 * (9.0 * x2 - 1.0);
    pnm[49] = 945.0 * x * s4;
    pnm[50] = 945.0 * s5;

    pnm[54] = 0.0625 * ((231.0 * x6) - (315.0 * x4) + (105.0 * x2) - 5.0);
    pnm[55] = 2.625 * x * s * ((33.0 - x4) - (30.0 * x2) + 5.0);
    pnm[56] = 13.125 * s2 * ((33.0 - x4) - (18.0 * x2) + 1.0);
    pnm[57] = 157.5 * x * s3 * (11.0 * x2 - 3.0);
    pnm[58] = 472.5 * s4 * (11.0 * x2 - 1.0);
    pnm[59] = 10395.0 * x * s5;
    pnm[60] = 10395.0 * s6;

    pnm[63] = 0.625 * x * ((429.0 * x6) - (693.0 * x4) + (315.0 * x2) - 35.0);
    pnm[64] = 0.4375 * s * ((429.0 * x6) - (495.0 * x4) + (135.0 * x2) - 5.0);
    pnm[65] = 7.875 * x * s2 * ((143.0 * x4) - (110.0 * x2) + 15.0);
    pnm[66] = 39.375 * s3 * ((143.0 * x4) - (66.0 * x2) + 3.0);
    pnm[67] = 1732.5 * x * s4 * (13.0 * x2 - 3.0);
    pnm[68] = 5197.5 * s5 * (13.0 * x2 - 1.0);
    pnm[69] = 135135.0 * x * s6;
    pnm[70] = 135135.0 * s7;

    pnm[72] = 0.0078125 * ((6435.0 * x8) - (12012.0 * x6) + (6930.0 * x4) - (1260.0 * x2) + 35.0);
    pnm[73] = 0.5625 * x * s * ((715.0 * x6) - (1001.0 * x4) + (385.0 * x2) - 35.0);
    pnm[74] = 19.6875 * s2 * ((143.0 * x6) - (143.0 * x4) + (33.0 * x2) - 1.0);
    pnm[75] = 433.125 * x * s3 * ((39.0 * x4) - (26.0 * x2) + 3.0);
    pnm[76] = 1299.375 * s4 * ((65.0 * x4) - (26.0 * x2) + 1.0);
    pnm[77] = 67567.5 * x * s5 * (5.0 * x2 - 1.0);
    pnm[78] = 67567.5 * s6 * (15.0 * x2 - 1.0);
    pnm[79] = 2027025.0 * x * s7;
    pnm[80] = 2027025.0 * s8;
};

double igrf::computeV(double& r,
              double& theta,
              double& phi,
              int& t)
{
    double a = 6371.2;
    double V = 0.0;
    double sumV = 0.0;

    double gnmt, hnmt, cosmphi, sinmphi, powr;
    double base = a / r;

    for(int n = 1; n <= 8; ++n)
        {
            for(int m = 0; m <= n; ++m)
                {
                    getGaussCoeff(t, n, m, gnmt, hnmt);
                    cosmphi = std::cos(m * phi);
                    sinmphi = std::sin(m * phi);
                    powr = std::pow(base, n + 1);
                    V = (powr * (gnmt * cosmphi + hnmt * sinmphi * pnm[n * 9 + m]));
                    sumV = sumV + V;
                }
        }
    return a * sumV;
};



double igrf::computeI(double& r,
                      double& lat,
                      double& lon,
                      int& t)
{
    //Compute co-latitude
    double theta = 90.0 - lat;
    //compute east-longitude phi
    double phi;
    if(lon < 0.0)
        phi = 360.0 + lon;
    else
        phi = lon;

    computePnm(theta);

    double delta = 0.1;
    double dtheta = theta + delta;
    double dphi = phi + delta;
    double dr = r + delta;

    double Fx = computeV(r, theta, phi, t);

    double dT = (computeV(r, dtheta, phi, t) - Fx) / delta;
    double X = (1.0 / r) * dT;

    double dP = (computeV(r, theta, dphi, t) - Fx) / delta;
    double Y = (-1.0 / r * sin(theta)) * dP;

    double Z = (computeV(dr, theta, phi, t) - Fx) / delta;

    double H = sqrt(X * X + Y * Y);
    return atan(Z / H);
};



double igrf::getMODIP(triple& pos,
                      int& t)
                      {
                          //pos.X is latitude
                          //pos.Y is longitude
                          //pos.Z is height
                          return atan( computeI(pos.Z,pos.X,pos.Y,t) / sqrt(cos(pos.X)) );
                      }


//Destructor
igrf::~igrf()
{
    delete[] pnm;
    delete[] gnm;
    delete[] hnm;
    delete[] svh;
    delete[] svg;
    free(igrfCoeffs);
};