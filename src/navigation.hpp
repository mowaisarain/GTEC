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





#ifndef __NAVIGATION__
#define __NAVIGATION__


#include "internalTime.hpp"
#include <vector>
#include "ephemerisGE.hpp"
#include "ephemerisR.hpp"

/**
 * @class navigation
 * @author Muhammad Owais
 * @date 05/12/16
 * @file navigation.hpp
 * @brief This class navigation data.
 * 
 * This class defines navigation data, stored after reading RINEX 
 * navigation files, for different constellations.
 */
class navigation
{
    public:
    
        //!Member function read
        /*!Member function read parses input navigation files and constructs
        * internal navigation structure.
        */
        void read();
        
        //!Constructor with Input files
        /*!Constructs navigation object by reading input navigation files
        * defined by fnames.
        * \param fnames Vector of Navigation file names.
        */
        navigation(std::vector<std::string > fnames);

        std::vector<std::string > fileNames; //!< list of file names to read from

        float version;  //!< Stores RINEX version
        int leapSeconds;    //!< Stores leapSeconds from Navigation files
        

        //!Vector to store objects of type ephemerisGE for GPS
        std::vector< std::vector<ephemerisGE > > ephemeris_G;
        //!Vector to store objects of type ephemerisGE for Galileo
        std::vector< std::vector<ephemerisGE > > ephemeris_E;
        //!Vector to store objects of type ephemerisR for GLONASS
        std::vector< std::vector<ephemerisR > > ephemeris_R;
        //!Vector to store objects of type ephemerisGE for BeiDou
        std::vector< std::vector<ephemerisGE > > ephemeris_C;

    private:
        navigation();
        /**< Default Constructor. 
        * Hidden, cannot be used.
        */

};

#endif