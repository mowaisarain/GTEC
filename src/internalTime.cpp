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


#include "internalTime.hpp"

internalTime::internalTime()
{
	year = 0;
	month = 0;
	day = 0;
	hour = 0;
	minute = 0;
	second = 0;
	UNIX = 0;
};


internalTime::internalTime(int Y, int M, int D, int h, int m, int s)
{
	year = Y;
	month = M;
	day = D;
	hour = h;
	minute = m;
	second = s;
};


void internalTime::parse(std::string str)
{
	str = str.substr(1);

	size_t pos;
	year = std::stoi(str,&pos);
	str = str.substr(pos);
	month = std::stoi(str,&pos);
	str = str.substr(pos);
	day = std::stoi(str,&pos);
	str = str.substr(pos);
	hour = std::stoi(str,&pos);
	str = str.substr(pos);
	minute = std::stoi(str,&pos);
	str = str.substr(pos);
	second = std::stoi(str,&pos);
};


//This routine also returns remaining string after conversion
void internalTime::parse(std::string str, std::string &sys)
{
	str = str.substr(1);
	float s;

	size_t pos;
	year = std::stoi(str,&pos);
	str = str.substr(pos);
	month = std::stoi(str,&pos);
	str = str.substr(pos);
	day = std::stoi(str,&pos);
	str = str.substr(pos);
	hour = std::stoi(str,&pos);
	str = str.substr(pos);
	minute = std::stoi(str,&pos);
	str = str.substr(pos);
	s = std::stof(str,&pos);
	second = int(s);
	sys = str.substr(pos);
};


void internalTime::toUNIXTime()
{
	//Epoch for Unix internalTime is January 01, 1970, midnight UTC/GMT
	//first count year difference
	int monthDays[12] = {31,59,90,120,151,181,212,243,273,304,334,365};
	int years_elapsed = year - 1970;
	int UNIXTime = years_elapsed * 365 * 24 * 60 * 60;
	UNIXTime += monthDays[month-2] * 24 * 60 * 60;
	UNIXTime += (day - 1) * 24 * 60 * 60;
	UNIXTime += hour * 60 * 60;
	UNIXTime += minute * 60;
	UNIXTime += second;
	//Now this internalTime should be added with leap days since 1970 due to leap years
	int leapCount = 0;
	bool isLeap = false;
	for(int y = 1970; y<=year; ++y)
	{
		if ( (y%4 == 0) && (!(y%100 == 0)) )
			isLeap = true;

		if ( (y%4 == 0) && (y%100 == 0) && (y%400 == 0) )
			isLeap = true;
		if(isLeap)
			leapCount += 1;
	}
	UNIXTime += leapCount * 24 * 60 * 60;
	//This is standard UNIX internalTime without leap seconds
	
	UNIX = UNIXTime;
};


void internalTime::toUNIXTime(int leapSeconds)
{
	//Epoch for Unix internalTime is January 01, 1970, midnight UTC/GMT
	//first count year difference
	int monthDays[12] = {31,59,90,120,151,181,212,243,273,304,334,365};
	int years_elapsed = year - 1970;
	int UNIXTime = years_elapsed * 365 * 24 * 60 * 60;
	UNIXTime += monthDays[month-2] * 24 * 60 * 60;
	UNIXTime += (day - 1) * 24 * 60 * 60;
	UNIXTime += hour * 60 * 60;
	UNIXTime += minute * 60;
	UNIXTime += second;
	//Now this internalTime should be added with leap days since 1970 due to leap years
	int leapCount = 0;
	bool isLeap = false;
	for(int y = 1970; y<=year; ++y)
	{
		if ( (y%4 == 0) && (!(y%100 == 0)) )
			isLeap = true;

		if ( (y%4 == 0) && (y%100 == 0) && (y%400 == 0) )
			isLeap = true;
		if(isLeap)
			leapCount += 1;
	}
	UNIXTime += leapCount * 24 * 60 * 60;
	//This is standard UNIX internalTime without leap seconds
	//Adding leap seconds
	UNIX = UNIXTime + leapSeconds;
};

