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


#ifndef __PTR_PAIR__
#define __PTR_PAIR__


/**
 * @class ptr_pair
 * @author Muhammad Owais
 * @date 05/12/16
 * @file ptr_pair.hpp
 * @brief Class defining pointer pairs.
 * 
 * This Class Defines pointer pairs objects used in preprocessing to define arcs. Each arc coud be defined
 * as a @ref ptr_pair object having a start pointer (pointer to first value in arc) and an 
 * end pointer (pointer to last value in arc).
 */
class ptr_pair
{
  public:
    
    float* start; //!< Start pointer
    float* end;  //!< End pointer
    
    //!Default constructor.
    /*!Default constructur, creates @ref ptr_pair object with NULLL start and end pointers.
     */
    ptr_pair();
    
    //!Custom constructor.
    /*!Constructur, creates @ref ptr_pair object with start and end pointers set to given pointers.
     * @param s Input start pointer for new @ref ptr_pair object.
     * @param e Input end pointer for new @ref ptr_pair object.
     */
    ptr_pair(float* s, float* e);
};

#endif