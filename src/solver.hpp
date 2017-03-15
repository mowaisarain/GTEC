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


#ifndef __SOLVER__
#define __SOLVER__


#include "ObsData.hpp"
#include "navigation.hpp"
#include "constants.hpp"


/**
 * @class solver
 * @author Muhammad Owais
 * @date 15/03/17
 * @file system.hpp
 * @brief Class defining system assembly.
 * 
 * This Class Defines how system would be assembled from observation and navigation data. This class also defines 
 * the final system solver to be used.  
 */

class solver
{
    public:
	
	
	//!Constructor with observation/navigation data and system options.
        /*!Constructs solver object for linear system assembly, given an observation data (@ref ObsData), 
	 * navigation data (@ref navigation), and system options (@ref solutionMethod).
	 * @param odata Observation data (@ref ObsData).
	 * @param ndata Navigation data (@ref navigation).
	 * @param solType Option defining solution method (@ref solutionMethod).
	 */
        solver(ObsData& odata, navigation& ndata, solutionMethod solType);


	
	//!Constructor with observation/navigation data and with default system options.
        /*!Constructs solver object for linear system assembly, given an observation data (@ref ObsData), 
	 * navigation data (@ref navigation), and system options set to defaults (using general LU decomposition).
	 * @param odata Observation data (@ref ObsData).
	 * @param ndata Navigation data (@ref navigation).
	 */
        solver(ObsData& odata, navigation& ndata);
	
	
	//! Pointer to @ref ObsData object.
	ObsData* od; 
	
	//! Pointer to @ref navigation object.
	navigation* nd; 

	
        //! Stores matrix B.
        /*! This is stored matrix B. B is a boolean matrix relating each value in 
         * vector S to a given arc number. The \f$ i^{th} \f$ row of B has only one 
         * non-zero in the \f$ j^{th} \f$ column, relating \f$ i^{th} \f$ value in vector S
         * to \f$ j^{th} \f$ arc number defined by @ref S_arcnum. Size of B is 
         * ( @ref size_of_S \f$ x \f$ @ref numArcs ).
         */
        double* B;

        //! Builds matrix B.
        /*! This function builds and stores matrix B.
         */
        void buildB();
	

	//!Clean-up function.
        /*!This function cleans up internal workspace, should be called before the end 
         * of object's lifetime.
	 */
        void cleanUp();


    private:
      
        solver(); //!< default hidden Constructor 
	    solutionMethod method; //!< @ref solutionMethod enumeration defining type of solution.       


};

#endif









