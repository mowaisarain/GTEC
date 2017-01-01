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