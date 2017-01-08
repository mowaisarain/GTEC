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



#include <iostream>
#include <fstream>
#include "triple.hpp"
#include "constants.hpp"
#include "navigation.hpp"
#include "inout.hpp"
#include "ObsData.hpp"



int main(int argc, char* argv[])
{    
    
    inout io;
    io.process_Inputs(argc,argv);
    io.dump(std::cout);

    int samplingtime = 10; //in Minutes
    
	ObsData obs(io.obsfiles,io.satSys);
	obs.read();
    std::cout << "done reading all files..\n";


    obs.pre_process(io.minArcLen,io.intrpolIntrvl,io.deg);
    std::cout << "done preprocessing..\n";

  
    navigation navdata(io.navfiles);
    navdata.read();


//    int rej_hours = 12; //Number of hours to reject from start and end
//    int start = (rej_hours * 60 * 60 ) / observations.interval;
//    int end = observations.timeline_main.size() - start;
//    for(int i=start;i<end;++i)
//    { 
//        
//    }



//    
//    std::cout << observations.size_of_S << "\n";
//    std::cout << observations.arcs3.size() << "\n";
//    std::cout << observations.ASP3.size() << "\n";
//    std::cout << observations.Aprn3.size() << "\n";
//    
    
    
//    int arcnt = 0;
//    for (auto arc: observations.arcs3)
//    {
//        
//        std::cout << arc.start << " " << arc.end << " " << observations.ASP3[arcnt] <<
//        " " << observations.Aprn3[arcnt] << "\n";
//        arcnt += 1;
//    }


//
//    //build Naive S
//    std::cout << "building Naive S..\n";
//    int start = (12 * 60 * 60) / observations.interval;
//    int end = observations.timeline_main.size() - start;
//    int ae = observations.arcs3.size();
//    std::cout << start << " " << end << " " << observations.timeline_main.size() << "\n";
//    int i,j;
//    float* tmptr = NULL;
//
//    for(i = start; i<end; ++i)
//    {
//        //use this as index across all prn to get data for ith epoch
//        for (auto arc : observations.GPS_ucTEC)
//        {
//            //there is no need to check for non-zeros
//            //as arcs3 has all non zero
//            for (j=0; j<ae; ++j)
//            {
//                tmptr = reinterpret_cast<float*> (&arc[i]);
//                if(  ( tmptr >= observations.arcs3[j].start ) && ( tmptr < observations.arcs3[j].end ) )
//                {
//                    //Now this value is a non zero and belongs to 
//                    //jth arc_Number and jth prn_id
//                    //push this info in respective vectors
//                    observations.S.push_back(arc[i]);
//                    //arc numbers start from zero '0'
//                    //observations.S_arcnum.push_back(j);
//                    //prn IDs are in the range [1-120]G32+R24+E30+C34
//                    //observations.S_prn.push_back(observations.Aprn3[j]);
//                }
//            }
//        }
//        
//        //std::cout << "<<<<< " << i <<  " >>>>>>>>\n";
//    }
//    std::cout << "Size of S: " << observations.S.size() << "\n";
//
//std::ofstream file1, file2,file3,file4;
//file1.open("file1.txt");
//file2.open("file2.txt");
//file3.open("file3.txt");
//file4.open("file4.txt");
//
//    float* ptr1;
//    float* ptr2;
//    ptr1 = reinterpret_cast<float*> ( &*observations.GPS_ucTEC[0].begin() );
//    ptr2 = reinterpret_cast<float*> ( &*(observations.GPS_ucTEC[0].end()-1) );
//    
//    
//    while(ptr1 <= ptr2)
//    {
//        file1 << *ptr1 << "\n";
//        file2 << ptr1 << "\n";
//        ptr1 += 1;
//    }
//    
//    
//    
//    for(auto itr = observations.GPS_ucTEC[0].begin(); itr < observations.GPS_ucTEC[0].end(); ++itr )
//    {
//        file3 << *itr << "\n";
//        file4 << reinterpret_cast<float*> (&*itr) << "\n";
//    }
//       
//
//
//
//
// 
//file1.close();
//file2.close();
//file3.close();
//file4.close();














    int rej_hours = 12; //Number of hours to reject from start and end
    int istart = (rej_hours * 60 * 60 ) / obs.interval;
    int iend = obs.timeline_main.size() - istart;


    //start Marking
    bool started = false;
    int startidx = 0;
    int endidx = 0;
    int minimum = (2 * 60 * 60) / obs.interval;
    int prnid = 0;
    for(auto prn : obs.GPS_ucTEC)
    {
        prnid += 1;
        started = false;
        startidx = 0;
        endidx = 0;      
        for(int i=istart;i<iend;++i)
        {
            if(prn[i] != 0.0)
                if(started)
                    continue;
                else
                {
                    started = true;
                    startidx = i;
                }
            else
                if(started)
                {
                    endidx = i-1;
                    if((i - startidx) >= minimum )
                    {
                        obs.intse.push_back(int_pair(startidx,endidx));
                        obs.prnid.push_back(prnid);
                    }
                    started = false;
                    startidx = endidx = 0;
                }
        }
    }
    
    //For GLONASS
    for(auto prn : obs.GLO_ucTEC)
    {
        prnid += 1;
        started = false;
        startidx = 0;
        endidx = 0;      
        for(int i=istart;i<iend;++i)
        {
            if(prn[i] != 0.0)
                if(started)
                    continue;
                else
                {
                    started = true;
                    startidx = i;
                }
            else
                if(started)
                {
                    endidx = i-1;
                    if((i - startidx) >= minimum )
                    {
                        obs.intse.push_back(int_pair(startidx,endidx));
                        obs.prnid.push_back(prnid);
                    }
                    started = false;
                    startidx = endidx = 0;
                }
        }
    }    
    
    //For GALILEO
    for(auto prn : obs.GAL_ucTEC)
    {
        prnid += 1;
        started = false;
        startidx = 0;
        endidx = 0;      
        for(int i=istart;i<iend;++i)
        {
            if(prn[i] != 0.0)
                if(started)
                    continue;
                else
                {
                    started = true;
                    startidx = i;
                }
            else
                if(started)
                {
                    endidx = i-1;
                    if((i - startidx) >= minimum )
                    {
                        obs.intse.push_back(int_pair(startidx,endidx));
                        obs.prnid.push_back(prnid);
                    }
                    started = false;
                    startidx = endidx = 0;
                }
        }
    }    
    
    //For BEIDOU
    for(auto prn : obs.BDU_ucTEC)
    {
        prnid += 1;
        started = false;
        startidx = 0;
        endidx = 0;      
        for(int i=istart;i<iend;++i)
        {
            if(prn[i] != 0.0)
                if(started)
                    continue;
                else
                {
                    started = true;
                    startidx = i;
                }
            else
                if(started)
                {
                    endidx = i-1;
                    if((i - startidx) >= minimum )
                    {
                        obs.intse.push_back(int_pair(startidx,endidx));
                        obs.prnid.push_back(prnid);
                    }
                    started = false;
                    startidx = endidx = 0;
                }
        }
    }
    
    
    //building S
    std::cout << "building S..\n";
    obs.numArcs = obs.intse.size();
    int i,j;
    float* tmptr = NULL;
    int ecount = 0;
    //number of epochs in sampling time
    int nepochs_st = (samplingtime * 60) / obs.interval;  
    int st = 0; //nth sampling time
    
    float poly2d[9];
    poly2d[0]=0.5; poly2d[1]=0.5; poly2d[2]=0.5; poly2d[3]=0.5; poly2d[4]=0.5;
    poly2d[5]=0.5; poly2d[6]=0.5; poly2d[7]=0.5; poly2d[8]=0.5; 
    
    int Sstart = 0;

    for(i = istart; i<iend; ++i)
    {
        ecount += 1;
        //use this as index across all prn to get data for ith epoch
        prnid = 0;
        for (auto arc : obs.GPS_ucTEC)
        {
            prnid +=1;
            for (j=0; j<obs.numArcs; ++j)
            {
                if( obs.prnid[j] == prnid )
                {
                    if(i >= obs.intse[j].start && i <= obs.intse[j].end )
                    {
                        //Now this value is a non zero and belongs to 
                        //jth arc_Number and jth prn_id
                        //push this info in respective vectors
                        obs.S.push_back(arc[i]);
                        //arc numbers start from zero '0'
                        obs.S_arcnum.push_back(j);
                        //prn IDs are in the range [1-120]G32+R24+E30+C34
                        obs.S_prn.push_back(prnid);
                    }
                }
            }
        }        
        
        for (auto arc : obs.GLO_ucTEC)
        {
            prnid +=1;
            for (j=0; j<obs.numArcs; ++j)
            {
                if( obs.prnid[j] == prnid )
                {
                    if(i >= obs.intse[j].start && i <= obs.intse[j].end )
                    {
                        //Now this value is a non zero and belongs to 
                        //jth arc_Number and jth prn_id
                        //push this info in respective vectors
                        obs.S.push_back(arc[i]);
                        //arc numbers start from zero '0'
                        obs.S_arcnum.push_back(j);
                        //prn IDs are in the range [1-120]G32+R24+E30+C34
                        obs.S_prn.push_back(prnid);
                    }
                }
            }
        }        
        for (auto arc : obs.GAL_ucTEC)
        {
            prnid +=1;
            for (j=0; j<obs.numArcs; ++j)
            {
                if( obs.prnid[j] == prnid )
                {
                    if(i >= obs.intse[j].start && i <= obs.intse[j].end )
                    {
                        //Now this value is a non zero and belongs to 
                        //jth arc_Number and jth prn_id
                        //push this info in respective vectors
                        obs.S.push_back(arc[i]);
                        //arc numbers start from zero '0'
                        obs.S_arcnum.push_back(j);
                        //prn IDs are in the range [1-120]G32+R24+E30+C34
                        obs.S_prn.push_back(prnid);
                    }
                }
            }
        }        
        for (auto arc : obs.BDU_ucTEC)
        {
            prnid +=1;
            for (j=0; j<obs.numArcs; ++j)
            {
                if( obs.prnid[j] == prnid )
                {
                    if(i >= obs.intse[j].start && i <= obs.intse[j].end )
                    {
                        //Now this value is a non zero and belongs to 
                        //jth arc_Number and jth prn_id
                        //push this info in respective vectors
                        obs.S.push_back(arc[i]);
                        //arc numbers start from zero '0'
                        obs.S_arcnum.push_back(j);
                        //prn IDs are in the range [1-120]G32+R24+E30+C34
                        obs.S_prn.push_back(prnid);
                    }
                }
            }
        }
        
        
        if(ecount == nepochs_st)
        {
            //Build block of A and partial B and solve for b
            //loop over S but partial
            for(int si=Sstart;si<obs.S.size();++si)
            {
                //calculate 9 floats for block of A
                //for this use time ??
                
                //store floats
                
                
                

            }
            
                //std::cout << "ST: " << st
                //<< " Sstart: " << Sstart
                //<< " S_Size: " << obs.S.size()
                //<< " Block_S: " << obs.S.size()-Sstart << "\n";
            
            
            //Done for this Sampling time reset variables
            st += 1;//increment sampling time counter 0 is the 1st sampling time
            ecount = 0; //reset epoch counter
            Sstart = obs.S.size();
        }
        
        
        //std::cout << "<<<<< " << i <<  " >>>>>>>>\n";
    }
    
    std::cout << "Size of S: " << obs.S.size() << " Number of Arcs: " << obs.numArcs << "\n";
    
    
//    std::cout << obs.intse.size() << " " << obs.prnid.size() <<  "\n";
//    
//    
//    for (int i=0;i<obs.intse.size();++i)
//    {
//        std::cout << obs.intse[i].start << "  " << obs.intse[i].end << "  "
//        << obs.prnid[i] << "\n";
//    }


    //building B
    std::cout << "building matrix B..\n";
    obs.buildB();
    
    obs.cleanUp();
	return 0;
}