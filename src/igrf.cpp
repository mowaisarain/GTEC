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


std::vector<std::string> linesplit(std::string str)
{
	std::string field = "";
	char tmp;
	size_t fillcount = 1;
	std::vector<std::string> str_vec;
	for (int i=0; i<str.length(); ++i)
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
			field.append(fillcount,tmp);
		}
	}
	str_vec.push_back(field);
	return str_vec;
};


int getGaussCoeffIGRF( std::string fname, float* gnm, float* hnm, float* svg, float* svh )
{
	std::ifstream igrf_file;
	igrf_file.open(fname);
	std::string line;
	int linecount = 0;
	std::vector<std::string> fields;
	std::vector<std::string> years;
	
	int n,m,t;

	if (igrf_file.is_open())
	{
		while ( getline (igrf_file,line) )
		{
			linecount += 1;
			fields = linesplit(line);

			if(linecount > 4)
			{
				n = stoi(fields[1]);
				m = stoi(fields[2]);
				if(fields[0] == "g")
					{
						for (std::pair<std::vector<std::string>::iterator, std::vector<std::string>::iterator>
						i(years.begin()+3, fields.begin()+3);
						i.first != years.end()-1  && i.second != fields.end()-1;
						++i.first, ++i.second)
						{
							t = std::stoi(*i.first) - 1900;
							gnm[n*mdim*tdim + m*tdim + t] = std::stof(*i.second);
					
						}
						svg[n*mdim + m] = std::stof(fields.back());
					}
				else if(fields[0] == "h")
					{
						for (std::pair<std::vector<std::string>::iterator, std::vector<std::string>::iterator>
						i(years.begin()+3, fields.begin()+3);
						i.first != years.end()-1  && i.second != fields.end()-1;
						++i.first, ++i.second)
						{
							t = std::stoi(*i.first) - 1900;
							hnm[n*mdim*tdim + m*tdim + t] = std::stof(*i.second);
					
						}
						svh[n*mdim + m] = std::stof(fields.back());
					}
	 
			}
			else if (linecount == 4)
			{
				for (std::vector<std::string>::iterator it = fields.begin() ; it != fields.end(); ++it)
					years.push_back(*it);
			}
		}
		igrf_file.close();
		return 0;
	}
	else 
	{
		std::cout << "Unable to open igrf file\n";
		return -1; 
	}

};




void getGaussCoeff(const float* gnm,
		   const float* hnm, 
		   const float* svg,
		   const float* svh, 
		   const int &t, 
		   const int &n, 
		   const int &m, 
		   float &gnmt, 
		   float &hnmt )
{

	if(t < 1900 || t > 2020) { std::cout << "Invalid year for IGRF12\n"; exit(-1); }

	int ti = t - 1900;
	int T0 = ti - (ti % 5);
	int T05 = T0 + 5;

	if(ti == 115)
	{
		gnmt = gnm[n*mdim*tdim + m*tdim + ti];
		hnmt = hnm[n*mdim*tdim + m*tdim + ti];
	}
	else if(ti > 115)
	{
		gnmt = svg[n*mdim + m];
		hnmt = svh[n*mdim + m];
	}
	else
	{
		gnmt = ( gnm[n*mdim*tdim + m*tdim + T05] - gnm[n*mdim*tdim + m*tdim + T0] ) / 5;
		hnmt = ( hnm[n*mdim*tdim + m*tdim + T05] - hnm[n*mdim*tdim + m*tdim + T0] ) / 5;
	}
};




int checkInput(const float* gnm, const float* hnm, const float* svg, const float* svh)
{
	float hnm331900 = 523;
	float gnm321955 = 1288;
	float svg43 = 4.1;
	float svh53 = -1.2;
	//float one = hnm[3][3][0];
	float one = hnm[3*mdim*tdim + 3*tdim + 0];	

	//float two = gnm[3][2][55];
	float two = gnm[3*mdim*tdim + 2*tdim + 55];	

	//float three = svg[4][3];
	float three = svg[4*mdim + 3];

	//float four = svh[5][3];
	float four = svh[5*mdim + 3];

	if( (hnm331900 == one)  && (gnm321955 == two) && (svg43 == three) && (svh53 == four) )
	{
		//std::cout << "SUCCESS\n";
		return 0;
	}
	else
	{
		std::cout << hnm331900 << " " << one << "\n";
		std::cout << gnm321955 << " " << two << "\n";
		std::cout << svg43 << " " << three << "\n";
		std::cout << svh53 << " " << four << "\n";
		std::cout << "FAIL\n";
		return -1;
	}
};



void calculatePnm(float &theta, float* pnm )
{
//Reference: http://ciks.cbt.nist.gov/~garbocz/paper134/node10.html

	float x = std::cos(theta);
	float s = std::sqrt(1.0 - (x*x));

	float x2 = x*x;
	float x3 = x2*x;
	float x4 = x3*x;
	float x5 = x4*x;
	float x6 = x5*x;
	float x8 = x6 * x * x;

	float s2 = s*s;
	float s3 = s2*s;
	float s4 = s3*s;
	float s5 = s4*s;
	float s6 = s5*s;
	float s7 = s6*s;
	float s8 = s6*s;


	pnm[0] = 1.0;

	pnm[9] = x;
	pnm[10] = s;

	pnm[18] = 0.5 * ((3.0 * x2 )-1);
	pnm[19] = 3.0 * x * s;
	pnm[20] = 3.0 * (1.0 - x2);

	pnm[27] = 0.5 * x * ( (5.0 * x2) - 3.0 );
	pnm[28] = 1.5 * ( (5.0 * x2) - 1.0 ) * s;
	pnm[29] = 15.0 * x * (1.0 - x2);
	pnm[30] = 15.0 * s3;

	pnm[36] = 0.125 * ( (35.0 * x4) - (30.0 * x2) + 3.0 );
	pnm[37] = 2.5 * ( (7.0 * x3) - (3.0 * x) );
	pnm[38] = 7.5 * (7.0 * x2 - 1)*(1.0 - x2);
	pnm[39] = 105.0 * x * s3;
	pnm[40] = 105.0 * s4;

	pnm[45] = 0.125 * x * ( (63.0 * x4)-(70.0 * x2)+15.0 );
	pnm[46] = 1.875 * s * ( (21.0 * x4)-(14.0 * x2)+1.0 );
	pnm[47] = 52.5 * x * ( 1.0 - x2 )*(3.0 * x2 - 1.0);
	pnm[48] = 52.5 * s3 * (9.0 * x2 - 1.0);
	pnm[49] = 945.0 * x * s4;
	pnm[50] = 945.0 * s5;

	pnm[54] = 0.0625 * ( (231.0 * x6)-(315.0 * x4)+(105.0 * x2)-5.0 );
	pnm[55] = 2.625 * x * s * ( ( 33.0 - x4 )-(30.0 * x2) + 5.0 );
	pnm[56] = 13.125 * s2 * ( ( 33.0 - x4 )-(18.0 * x2) + 1.0 );
	pnm[57] = 157.5 * x * s3 * ( 11.0 * x2 - 3.0 );
	pnm[58] = 472.5 * s4 * ( 11.0 * x2 - 1.0 );
	pnm[59] = 10395.0 * x * s5;
	pnm[60] = 10395.0 * s6;

	pnm[63] = 0.625 * x * ( (429.0 * x6)-(693.0 * x4)+(315.0 * x2)-35.0 );
	pnm[64] = 0.4375 * s * ( (429.0 * x6)-(495.0 * x4)+(135.0 * x2)-5.0 );
	pnm[65] = 7.875 * x * s2 * ( (143.0 * x4)-(110.0 * x2)+15.0 );
	pnm[66] = 39.375 * s3 * ( (143.0 * x4)-(66.0 * x2)+3.0 );
	pnm[67] = 1732.5 * x * s4 * ( 13.0 * x2 - 3.0 );
	pnm[68] = 5197.5  * s5 * ( 13.0 * x2 - 1.0 );
	pnm[69] = 135135.0 * x * s6;
	pnm[70] = 135135.0 * s7;

	pnm[72] = 0.0078125 * ( (6435.0 * x8)-(12012.0 * x6)+(6930.0 * x4)-(1260.0 * x2) + 35.0 );
	pnm[73] = 0.5625 * x * s * ( (715.0 * x6)-(1001.0 * x4)+(385.0 * x2) - 35.0 ); 
	pnm[74] = 19.6875 * s2 * ( (143.0 * x6)-(143.0 * x4)+(33.0 * x2) - 1.0 );
	pnm[75] = 433.125 * x * s3 * ( (39.0 * x4) - (26.0 * x2) + 3.0 );
	pnm[76] = 1299.375 * s4 * ( (65.0 * x4) - (26.0 * x2) + 1.0 );
	pnm[77] = 67567.5 * x * s5 * ( 5.0 * x2 - 1.0 );
	pnm[78] = 67567.5 * s6 * ( 15.0 * x2 - 1.0 );
	pnm[79] = 2027025.0 * x * s7;
	pnm[80] = 2027025.0 * s8;
};



float computeV(const float &r, float &theta, const float &phi, int &t, 
	float *gnm, 
	float *hnm, 
	float *svg, 
	float *svh,
	float *pnm )
	{
		float a = 6371.2;
		float V = 0.0;
		float sumV = 0.0;

		float gnmt,hnmt,cosmphi,sinmphi,powr;
		float base = a/r;



		for (int n=1; n<=8; ++n)
		{
			for (int m=0; m<=n; ++m)
			{
				getGaussCoeff(gnm,hnm,svg,svh,t,n,m,gnmt,hnmt);
				cosmphi = std::cos(m*phi);
				sinmphi = std::sin(m*phi);
				powr = std::pow(base,n+1);
				V = ( powr * ( gnmt*cosmphi + hnmt*sinmphi*pnm[n*9 + m] ) );
				sumV = sumV + V;
			}
		}
		return sumV;
	};


float computeI(float &r, float &lat, float &lon, int &t,
	float *gnm, 
	float *hnm, 
	float *svg, 
	float *svh )
	{
		//Compute co-latitude
		float theta = 90.0 - lat;
		//compute east-longitude phi
		float phi;
		if(lon < 0.0)
			phi = 360.0 + lon;
		else
			phi = lon;

		float* pnm = new float[81];
		calculatePnm(theta, pnm);


		float delta = 0.1;
		float dtheta = theta + delta;
		float dphi = phi + delta;
		float dr = r + delta;

		float Fx = computeV(r,theta,phi,t,gnm,hnm,svg,svh,pnm);

		float dT = ( computeV(r,dtheta,phi,t,gnm,hnm,svg,svh,pnm) - Fx ) / delta;
		float X = (1.0 / r) * dT;
		
		float dP = ( computeV(r,theta,dphi,t,gnm,hnm,svg,svh,pnm) - Fx ) / delta;
		float Y = ( -1.0 / r * sin(theta) ) * dP;
		
		float Z = ( computeV(dr,theta,phi,t,gnm,hnm,svg,svh,pnm) - Fx ) / delta;
	
		float H = sqrt( X*X + Y*Y );
		float I = atan( Z/H );
		
		delete[] pnm;
		return I;
	};



