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





#ifndef TRIPLE_HPP
#define TRIPLE_HPP

#include <iostream>

/**
 * @class triple
 * @author Muhammad Owais
 * @date 05/12/16
 * @file triple.hpp
 * @brief This class defines a 3-D Coordinate
 */
class triple
{
  public:
	double X; //!< Stores X Coordinate
	double Y; //!< Stores Y Coordinate
	double Z; //!< Stores Z Coordinate
    
    //!Member function dump
    /*!Member function dump output coordinates into a given output stream.
     * \param s output stream
    */    
	void dump(std::ostream& s);
    
    
    
    //!Default constructor
    /*!Constructor creating triple object initialized to zero.
    */    
    triple();
    
    //!Custom constructor
    /*!Constructor creating triple object with three doubles given as input.
     * \param x  X Coordinate
     * \param y  Y Coordinate
     * \param z  Z Coordinate
    */    
    triple(const double& x, const double& y, const double& z);
};


#endif
