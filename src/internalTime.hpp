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





#ifndef __INTERNALTIME__HPP
#define __INTERNALTIME__HPP

#include <string>


/**
 * @class internalTime
 * @author Muhammad Owais
 * @date 04/12/16
 * @file internalTime.hpp
 * @brief Class defining internal time format.
 * 
 * This Class Defines Internal time which is based on Unix Time.
 * It stores the normal Date/Time as (Year,Month,Day,Hour,Minute,Second),
 * while also providing equivalent UNIX Time. An instance of this class
 * could be generated by explicitely providing normal Date/Time values or by
 * providing a string which would be parse to store time in both formats.
 */
class internalTime
{

  public:
	int year;
    /**<
     * Stores Year as Integer
     */
	int month;  //!< Stores Month as Integer
	int day;    //!< Stores day as Integer
	int hour;   //!< Stores hour as Integer
	int minute; //!< Stores minute as Integer
	int second; //!< Stores second as Integer
	int UNIX;   //!< Stores Converted UNIX Time as Integer



    //!Constructor with explicit values
    /*!Constructs internalTime object explicitly taking date/time values
     * as parameters. Requires 6 integers (YYYY,MM,DD,hh,mm,ss).
     * \param Y year(YYYY), given as integer
     * \param M Month(MM), given as integer
     * \param D Day(DD), given as integer
     * \param h Hour(hh), given as integer
     * \param m Minute(mm), given as integer
     * \param s Second(ss), given as integer
    */
    internalTime(int Y, int M, int D, int h, int m, int s); 



    //!Member function parse
    /*!Member function parse sets internal values by parsing a given
     * string representing date/time values.
     * \param strtime string representing time.
    */
	void parse(std::string strtime); //set internalTime object by parsing string




	//This routine also returns remaining string after conversion
	void parse(std::string, std::string&);


    //!Member Function, providing UNIX time.
    /*!Member function, converting stored time to UNIX time.
    */
	void toUNIXTime(); //sets corresponding UNIX internalTime
    
    
    
    
    void toUNIXTime(int); //sets corresponding UNIX internalTime + leapseconds
    
    internalTime(); 
    /**< Default Constructor. 
     */

};



#endif
