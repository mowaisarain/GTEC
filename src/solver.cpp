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
#include "triple.hpp"

//Constructor with system options
solver::solver(ObsData& odata, navigation& ndata, inout& in, solutionMethod solType)
{
  //set system options
  method = solType;
  
  //set ObsData and navigation pointers
  od = &odata;
  nd = &ndata;
  inp = &in;
};



//Constructor with default system options
solver::solver(ObsData& odata, navigation& ndata, inout& in)
{
  //set default system options
  method = GENERAL;
  
  //set ObsData and navigation pointers
  od = &odata;
  nd = &ndata;
  inp = &in;
};


//This function builds and stores B
void solver::buildB()
{
    //Build B
    B = (double*) calloc(od->size_of_S * od->numArcs, sizeof(double));
    for(int i=0; i < od->size_of_S; ++i)
    {
        B[ i*(od->numArcs) + S_arcnum[i] ] = 1.0;
    }
};


//This function cleans up internal workspace 
//should be called before end of object's lifetime.
void solver::cleanUp()
{
  //cleanUp workspace
  free(B);
  free(A);
};




void solver::buildS(int& samplingtime)
{
    od->numArcs = od->intse.size();
    int i,j;
    float* tmptr = NULL;
    int ecount = 0;
    //number of epochs in sampling time
    int nepochs_st = (samplingtime * 60) / od->interval;  
    int st = 0; //nth sampling time

    int Sstart = 0;
    int id = 0;
    int Scount = 0; //value count in vector S
    int numBlocks = 0; //sampling block count
    
    int timeDiff;
    triple satXYZ;
    triple satLatLon;
    triple ippXYZ;
    double zenith;
    
    for(i = od->istart; i < od->iend; ++i)
    {
        ecount += 1;
        //use this as index across all prn to get data for ith epoch
        id = 0;
        for (auto arc : od->GPS_ucTEC)
        {
            id +=1;
            for (j=0; j< od->numArcs; ++j)
            {
                if( od->prnid[j] == id )
                {
                    if(i >= od->intse[j].start && i <= od->intse[j].end )
                    {
                        //Now this value is a non zero and belongs to 
                        //jth arc_Number and jth prn_id
                        //push this info in respective vectors
                        S.push_back(arc[i]);
                        //arc numbers start from zero '0'
                        S_arcnum.push_back(j);
                        //prn IDs are in the range [1-120]G32+R24+E30+C34
                        S_prn.push_back(id);
			
			
			//Get position
			for(int n=0; n < nd->ephemeris_G[id-1].size() - 1 ; ++n)
			{
			  timeDiff =  od->timeline_main[i] - 
				      nd->ephemeris_G[id-1][n].Toc;
			  if ( timeDiff > 0 && timeDiff <= 7200) //2 hours
			  {
			    nd->getPositionGE(nd->ephemeris_G[id-1][n], 
					      timeDiff, 
					      satXYZ);
                //convert to Lat/Long
                nd->ecefToEllipsoidal(satXYZ,satLatLon);
                //Compute IPP and coschi
                nd->computeIPP(od->MarkerPosition, satXYZ, inp->rh, ippXYZ, zenith);
                
			  }
              
			}
			
			
			
                    }
                }
            }
        }        
        
        for (auto arc : od->GLO_ucTEC)
        {
            id +=1;
            for (j=0; j< od->numArcs; ++j)
            {
                if( od->prnid[j] == id )
                {
                    if(i >= od->intse[j].start && i <= od->intse[j].end )
                    {
                        //Now this value is a non zero and belongs to 
                        //jth arc_Number and jth prn_id
                        //push this info in respective vectors
                        S.push_back(arc[i]);
                        //arc numbers start from zero '0'
                        S_arcnum.push_back(j);
                        //prn IDs are in the range [1-120]G32+R24+E30+C34
                        S_prn.push_back(id);
                    }
                }
            }
        }       
        
        for (auto arc : od->GAL_ucTEC)
        {
            id +=1;
            for (j=0; j< od->numArcs; ++j)
            {
                if( od->prnid[j] == id )
                {
                    if(i >= od->intse[j].start && i <= od->intse[j].end )
                    {
                        //Now this value is a non zero and belongs to 
                        //jth arc_Number and jth prn_id
                        //push this info in respective vectors
                        S.push_back(arc[i]);
                        //arc numbers start from zero '0'
                        S_arcnum.push_back(j);
                        //prn IDs are in the range [1-120]G32+R24+E30+C34
                        S_prn.push_back(id);
                    }
                }
            }
        }   
        
        for (auto arc : od->BDU_ucTEC)
        {
            id +=1;
            for (j=0; j< od->numArcs; ++j)
            {
                if( od->prnid[j] == id )
                {
                    if(i >= od->intse[j].start && i <= od->intse[j].end )
                    {
                        //Now this value is a non zero and belongs to 
                        //jth arc_Number and jth prn_id
                        //push this info in respective vectors
                        S.push_back(arc[i]);
                        //arc numbers start from zero '0'
                        S_arcnum.push_back(j);
                        //prn IDs are in the range [1-120]G32+R24+E30+C34
                        S_prn.push_back(id);
                    }
                }
            }
        }
        
        if(ecount == nepochs_st)
        {
           
/*          //Done for this Sampling time reset variables
            st += 1;//increment sampling time counter 0 is the 1st sampling time
            ecount = 0; //reset epoch counter
            Sstart = S.size();*/
            
            SdimVec.push_back( S.size() - Scount );
            OffSetVec.push_back( Scount );
            Scount = S.size();
            if( SdimVec.back() > SdimbMax )
            {
                SdimbMax = SdimVec.back();
            }
            numBlocks += 1;
        }
        
    }
    
    
};




void solver::buildA(int numCoeffs)
{
    if (numCoeffs == 6)
    {
        //Allocate A
        A = (double*) malloc( S.size() * numCoeffs * sizeof(double) );
        
         
    }
    else if (numCoeffs == 9)
    {
        //9 coefficients polynomial
        std::cout << "Polynomial not defined for: numCoeffs = 9.\n";
        exit(1);
    }
    else 
    {
        std::cout << "Invalid Parameter: numCoeffs.\n";
        exit(1);
    }
};

