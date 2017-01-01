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
};


#endif
