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