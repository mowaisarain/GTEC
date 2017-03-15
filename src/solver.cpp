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



#include "solver.hpp"

//Constructor with system options
solver::solver(ObsData& odata, navigation& ndata, solutionMethod solType)
{
  //set system options
  method = solType;
  
  //set ObsData and navigation pointers
  od = &odata;
  nd = &ndata;
};



//Constructor with default system options
solver::solver(ObsData& odata, navigation& ndata)
{
  //set default system options
  method = GENERAL;
  
  //set ObsData and navigation pointers
  od = &odata;
  nd = &ndata;
};


//This function builds and stores B
void solver::buildB()
{
    //Build B
    B = (double*) calloc(od->size_of_S * od->numArcs, sizeof(double));
    for(int i=0; i < od->size_of_S; ++i)
    {
        B[ i*(od->numArcs) + od->S_arcnum[i] ] = 1.0;
    }
};


//This function cleans up internal workspace 
//should be called before end of object's lifetime.
void solver::cleanUp()
{
  //cleanUp workspace
  free(B);
};





