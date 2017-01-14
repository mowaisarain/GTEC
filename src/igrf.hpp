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



#ifndef __IGRF__HPP
#define __IGRF__HPP

#include <string>
#include <vector>
#include <cmath>




class igrf
{
    
public:

    igrf(std::string fname);
    
    double getMODIP(double& r,
                    double& lat,
                    double& lon,
                    int& t);

    
private:

    const int nDim = 15;
    const int mDim = 15;
    const int tDim = 200;
    const int pnmDim = 81;

    double* gnm;
    double* hnm;
    double* svg;
    double* svh;
    double* pnm;

    igrf();
    ~igrf();
    
    std::vector<std::string> linesplit(std::string str);
    
    void getGaussCoeff( const int &t, 
                        const int &n, 
                        const int &m, 
                        double &gnmt, 
                        double &hnmt );
    
    int checkInput();
    
    void computePnm(double& theta);

    double computeV(double& r,
                  double& theta,
                  double& phi,
                  int &t);
                  
    double computeI(double& r,
                    double& lat,
                    double& lon,
                    int& t);
    
};




#endif



