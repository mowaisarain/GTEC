/*
    GTEC -  A high performance standardized implementation of 
    Multi Constellation GNSS Derived TEC Calibration 
    (Model by T/ICT4D Lab ICTP).
    Copyright (C) 2016,2017 Muhammad Owais
    
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
 * @brief This is class navigation data.
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
    navigation(std::vector<std::string> fnames);

    std::vector<std::string> fileNames; //!< list of file names to read from

    float version;   //!< Stores RINEX version
    int leapSeconds; //!< Stores leapSeconds from Navigation files

    //!Vector to store objects of type ephemerisGE for GPS
    std::vector<std::vector<ephemerisGE> > ephemeris_G;
    //!Vector to store objects of type ephemerisGE for Galileo
    std::vector<std::vector<ephemerisGE> > ephemeris_E;
    //!Vector to store objects of type ephemerisR for GLONASS
    std::vector<std::vector<ephemerisR> > ephemeris_R;
    //!Vector to store objects of type ephemerisGE for BeiDou
    std::vector<std::vector<ephemerisGE> > ephemeris_C;

    //!Function to compute GLONASS satellite positions.
    /*!This function calculates GLONASS satellite coordinates given
        * an ephemerisR object, and a step size.
        * \param initialConditions ephemerisR object containing initial conditions.
        * \param h Integer step size for next coordinate.
        * \param pos triple object returned with computed coordinates.
        */
    void getPositionR(ephemerisR& initialConditions, int h, triple& pos);

    //!Function to compute GPS/Galileo/BeiDou satellite positions.
    /*!This function calculates GPS/Galileo/BeiDou satellite coordinates given
        * an ephemerisGE object and time for which coordinates are required.
        * \param initial ephemerisGE object containing initial Keplerian elements.
        * \param t Integer time for which coordinates are to be computed.
        * \param pos triple object returned with computed coordinates.
        */
    void getPositionGE(ephemerisGE& initial, int t, triple& pos);

    //!Function to convert ECEF to ellipsoidal coordinates.
    /*!This function converts ECEF cartesian coordinates \f$ (x,y,z) \f$ to
         * ellipsoidal coordinates \f$ (\varphi,\lambda,h) \f$ respectively lattitude, 
         * longitude, and height. 
        * \param ecef ECEF cartesian coordinates.
        * \param ellipsoid Output ellipsoidal coordinates \f$ (\varphi,\lambda,h) \f$ .
        */
    void ecefToEllipsoidal(const triple& ecef, triple& ellipsoid);

    
    
    
    //!Function to compute satellite elevation and azimuth.
    /*!This function computes satellite elevation and azimuth given marker (receiver station) position
	 * in ECEF and ellipsoidal coordinates and satellite position in ECEF coordinates. This function implements
	 * elevation/azimuth computation as described in 
     * <a href="http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates">
     * Transformations between ECEF and ENU coordinates 
     * J. Sanz Subirana, J.M. Juan Zornoza and M. Hernández-Pajares, 
     * Technical University of Catalonia, Spain.
     * </a>
	 * \param markerECEF ECEF cartesian coordinates for marker (receiver station) as a @ref triple object.
         * \param sat ellipsoidal coordinates for satellite as a @ref triple object.
         * \param markerEllip ellipsoidal coordinates for marker (receiver station) as a @ref triple object.
         * \param elevation Output satellite elevation.
         * \param azimuth Output satellite azimuth.
         */
    void satElevAzim(triple& markerECEF, triple& sat, triple& markerEllip, double& elevation, double& azimuth);
    
    
    
    
    //!Function to compute IPP and zenith angle over IPP.
    /*!This function computes IPP (Ionospheric Pierce Point) in ECEF cartesian coordinates 
     * and zenith angle over IPP using sphere-line euation. Reference height of ionosphere, 
     * marker (receiver station), and satellite position are given as inputs. 
     * An integer status is returned describing solution type.
	 * \param marker ECEF cartesian coordinates for marker (receiver station) as a @ref triple object.
         * \param sat ECEF cartesian coordinates for satellite as a @ref triple object.
         * \param rh Ionosphere reference height in Kilometers.
         * \param IPP Output ECEF cartesian coordinates for IPP as a @ref triple object.
         * \param coschi Output zenith angle over IPP.
         */    
    int computeIPP(const triple& marker, const triple& sat, const double& rh, triple& IPP, double& coschi);

    

private:
    navigation();
    /**< Default Constructor. 
        * Hidden, cannot be used.
        */

    //!Function to compute eccentricity anomaly Ek.
    /*!This Function computes eccentricity anomaly Ek by Solving (iteratively) 
        * the Kepler equation for the eccentricity anomaly, using 
        * Newton–Raphson method, Equation -->  Mk = Ek - ( e * Sin(Ek) )
        * \param M mean anomaly for reference time tk.
        * \param e eccentricity.
        */
    float eccAnomaly(float M, float e);

    //!This Function apply rotations around uk, ik and Lk.
    /*!This Function apply rotations around uk, ik and Lk, 
        * Rotation ==
        * | Xk |                           | rk |
        * | Yk |  =  R3(-Lk)R1(-ik)R3(-uk) | 0  |
        * | Zk |                           | 0  |
        * Wwhere R1 and R3 are the rotation matrices defined at:
        * http://www.navipedia.net/index.php/Transformation_between_Terrestrial_Frames
        * By Hernández-Pajares, Technical University of Catalonia, Spain.
        * \param Lk Longitude of the ascending node LAMBDAk.
        * \param ik Inclination of the orbital plane.
        * \param uk Argument of latitude.
        * \param rk Radial distance rk.
        * \param pos triple object returned with computed coordinates.
        */
    void applyRotations(float& Lk, float& ik, float& uk, float& rk, triple& pos);
};

#endif