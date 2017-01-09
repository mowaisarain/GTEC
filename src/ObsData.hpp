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


#ifndef __OBS_DATA__
#define __OBS_DATA__

#include <iostream>
#include <fstream>
#include <vector>
#include "internalTime.hpp"
#include "triple.hpp"
#include "ptr_pair.hpp"
#include "int_pair.hpp"


/**
 * @class ObsData
 * @author Muhammad Owais
 * @date 05/12/16
 * @file ObsData.hpp
 * @brief Class defining observation data.
 * 
 * This Class Defines observation data handling, including reading from observation
 * files and storing in internal data structure, the raw non-calibrated TEC from phase
 * observables. This class also includes preprocessing routines being applied to 
 * internal data structure, and allot of dump routines for debugging and ploting arc states.  
 */

class ObsData
{
    public:
	
	//!Member function read
        /*!Member function read parses input observation files and constructs
        * internal observation structure.
        */
        void read();
	
	//!Constructor with Input files, and system string
        /*!Constructs observation object by seting input observation file name 
	 * vector @ref fnames given file names and setting system flags given system string.
	 * @param fvec Vector of observation file names.
	 * @param sysString string (any combination of 'G','R','E','C') defining constellations being processed.
	 */
        ObsData(std::vector<std::string > fvec, std::string sysString);




	//!Clean-up function.
    /*!This function cleans up internal workspace, should be called before end 
     * of object's lifetime.
	 */
        void cleanUp();


	
        int dumpArc(char, int);
        int dumpArcByTime(char, int);
        int dumpArcBinary(char, int);
        int dumpSizes();
	
	
	
	//!Function to perform preprocessing.
        /*!This function performs preprocessing by filling gaps using lagrange
	 * interpolation and removing phase jumps using quartiles and Inter Quartile Range.
	 * @param minArcLen minimum data duration(Seconds) to consider an arc valid.
	 * @param intrpolIntrvl Maximum gap duration (Seconds) to interpolate.
	 * @param deg Degree of Interpolation, passed to @ref lagrangeInterpolation.
	 */
        void pre_process(int minArcLen, int intrpolIntrvl, int deg);
	
	
	
        void dumpNonZeroArcs();
	
	

	
	
	
        int dumpArcBinaryPtrsAll();
        int dumpArcValuePtrsAll();


        std::vector<std::string > fnames; //!< list of file names to read from

        //float MarkerPositionX;
        //float MarkerPositionY;
        //float MarkerPositionZ;
        
        triple MarkerPosition; //!< @ref triple Object to store receiver-station position
        
        float version; //!< Stores RINEX version of observation files
	
        int interval; //!< Interval between observations in data file

        // Flags to indicate whether Data file contains a constellation
        bool hasGPS; //!< Flag to indicate whether Data file contains GPS Data
        bool hasGLO; //!< Flag to indicate whether Data file contains GLONASS Data
        bool hasGAL; //!< Flag to indicate whether Data file contains Galileo Data
        bool hasBEI; //!< Flag to indicate whether Data file contains BeiDou Data

        // Flags to indicate whether a contellation be included in solution
        bool readGPS; //!< Flag to indicate whether to process GPS Data
        bool readGLO; //!< Flag to indicate whether to process GLONASS Data
        bool readGAL; //!< Flag to indicate whether to process Galileo Data
        bool readBEI; //!< Flag to indicate whether to process BeiDou Data

        //! Time of first observation flag.
        /*! Flag to indicate whether Time of first observation was present in
	 *  observation Header.
	 */
        bool hasTOFO;

        std::string TOFO_system; //!< Time system of first observation from observation Header.
        
        internalTime TOFO; //!< @ref internalTime Object to store Time of first observation

        std::vector<int> timeline_main;  //!< Integer vector to store epochs in UNIX time
        
        //! Vectors to store raw non-calibrated TEC for GPS Satellites. 
        /*! This is a Vector of float-vectors, where first index is the Satellite prn-id
	 *  and the second index is the raw non-calibrated TEC for GPS satellites
	 *  corresponding to the epoch index in @ref timeline_main. 
	 */
        std::vector< std::vector<float> > GPS_ucTEC;
	
        //! Vectors to store raw non-calibrated TEC for GLONASS Satellites. 
        /*! This is a Vector of float-vectors, where first index is the Satellite prn-id
	 *  and the second index is the raw non-calibrated TEC for GLONASS satellites
	 *  corresponding to the epoch index in @ref timeline_main. 
	 */
        std::vector< std::vector<float> > GLO_ucTEC;
	
        //! Vectors to store raw non-calibrated TEC for Galileo Satellites. 
        /*! This is a Vector of float-vectors, where first index is the Satellite prn-id
	 *  and the second index is the raw non-calibrated TEC for Galileo satellites
	 *  corresponding to the epoch index in @ref timeline_main. 
	 */
        std::vector< std::vector<float> > GAL_ucTEC;
	
        //! Vectors to store raw non-calibrated TEC for BeiDou Satellites. 
        /*! This is a Vector of float-vectors, where first index is the Satellite prn-id
	 *  and the second index is the raw non-calibrated TEC for BeiDou satellites
	 *  corresponding to the epoch index in @ref timeline_main. 
	 */
        std::vector< std::vector<float> > BDU_ucTEC;

	
        //std::vector<float*> ASP;
        //std::vector<int> Aprn;  
      
        //std::vector<float*> ASP3;
        //std::vector<int> Aprn3;
        
	
        //! Stores vector S (non-calibrated TEC).
        /*! This vector stores all computed non-calibrated TEC values, arranged by
	 *  epochs. This is the input vector given to the system solver.
	 */
        std::vector<double> S;
	
        //! Stores arc numbers for @ref S.
        /*! This vector stores for each element in @ref S , a corresponding value 
	 *  indicating the its arc number. Arc numbers are defined by pre_processing phase
	 *  using @ref pre_process.
	 */	
        std::vector<int> S_arcnum;
	
	//! Stores Satellite IDs for @ref S.
        /*! This vector stores for each element in @ref S , a corresponding value 
	 *  indicating the its Satellite ID.
	 */
        std::vector<int> S_prn;
        
        
        
        int size_of_S; //!< Indicates size of @ref S.
        
        std::vector<int_pair > intse;
	
	//! Indicates total number of arcs.
        /*! Indicates total number of arcs formed. Arc numbers are defined by pre_processing phase
	 *  using @ref pre_process.
	 */
        int numArcs;
        
        
    //! Stores matrix B.
    /*! This is stored matrix B. B is a boolean matrix relating each value in 
     * vector S to a given arc number. The \f$ i^{th} \f$ row of B has only one 
     * non-zero in the \f$ j^{th} \f$ column, relating \f$ i^{th} \f$ value in vector S
     * to \f$ j^{th} \f$ arc number defined by @ref S_arcnum. Size of B is 
     * ( @ref size_of_S \f$ x \f$ @ref numArcs ).
     */
        double* B;        

        
	std::vector<int> prnid;
        
            
        

        int GPS_Mark[32];
        int GLO_Mark[24];
        int GAL_Mark[30];
        int BDU_Mark[34];

        int NonZero_Mark[120];
        int numNonZeroArcs;
        
	
	//! Initial non-zero arc pinters.
        /*! @ref ptr_pair Object containing Initial non-zero arcs, without preprocessing
	 *  being applied.
	 */
        std::vector<ptr_pair> arcs;
	
	//! Arc pointers without zeros
        /*! @ref ptr_pair Object containing arcs, without leading and trailing zeros.
	 */
        std::vector<ptr_pair> arcs2;
        
        //! Arc pointers without gaps
        /*! @ref ptr_pair Object containing arcs, with gaps removoed by 
	 *  @ref lagrangeInterpolation and phase jumps removed. These are the processed 
	 *  Arcs.
	 * 
	 */
        std::vector<ptr_pair> arcs3;
        


    //! Builds matrix B.
    /*! This function builds and stores matrix B.
     */
        void buildB();
        
        
        
	//!Function to dump raw matrix.
    /*!This Function dumps raw matrix to standard output stream, usefull in
     * debugging purposes.
	 * @param mat pointer to stored matrix.
	 * @param dim1 First dimension of matrix (number of rows).
	 * @param dim2 Second dimension of matrix (number of columns).
	 */        
        void dumpRawMatrix(const double* mat, int& dim1, int& dim2);



    private:
      
        ObsData(); //!< default hidden Constructor 
	
	
	//! Sets system flags
        /*! This function sets system flags given @ref sysString, to indicate
	 *  which constellations are processed.
	 *  @param sysString string (any combination of 'G','R','E','C') indicating constellations being processed. 
	 */
        void setSysFlags(std::string sysString);
	
        int pad_zero(int);
        void resetMark();
        int pad_zero();
        void markNonZeroArcs(int, int);
        void getnumNonZeroArcs();
	
	//! Sets Arc pointers using @ref ptr_pair objects
        /*! This function sets Arc pointers to start/end pairs using
	 *  @ref ptr_pair, which serve as input arcs to preprocessing phase.
	 */
        void setArcStartEnd();
	
	//!Function to perform lagrange interpolation
        /*!This function performs lagrange Interpolation needed in preprocessing phase, given a
	 * required degree for interpolation.
	 * @param target pointer to the value being interpolated.
	 * @param s start pointer of the arc.
	 * @param e end pointer of the arc.
	 * @param deg degree of Interpolation.
	 */
        int lagrangeInterpolation(float* target, float* s, float* e, int deg);

};

#endif