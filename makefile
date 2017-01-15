######################################################################
#
#    GTEC -  A high performance standardized implementation of 
#    Multi Constellation GNSS Derived TEC Calibration 
#    (Model by T/ICT4D Lab ICTP).
#    Copyright (C) 2016,2017  Muhammad Owais
#    
#    This file is part of GTEC.
#
#    GTEC is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, version 2 of the License.
#
#    GTEC is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with GTEC.  If not, see <http://www.gnu.org/licenses/>.
#    
#    Disclaimer: GTEC is a research implementation which is under 
#    development and should not be considered fully functional unless 
#    otherwise stated or a release is announced. Author is providing this 
#    software on a best effort AS IS basis and do not warrant validity, 
#    functionality, and suitability for any particular purpose. All copyright 
#    notices must be kept intact. 
#
######################################################################



#------------------------------------------------------------------------------
CC      = g++
# BOOSTLIB  defines boost lib path
# BOOSTINC  defines boost include path
CFLAGS  = -std=c++11 -I$(BOOSTINC) -L$(BOOSTLIB)
LDFLAGS = -static -lboost_system -lboost_filesystem
PROGRAM = GTEC
SRCDIR  = src
OBJS = inout.o int_pair.o ptr_pair.o internalTime.o ObsData.o navigation.o triple.o igrf.o GTEC.o
SRCS = $(SRCDIR)/inout.cpp $(SRCDIR)/int_pair.cpp $(SRCDIR)/ptr_pair.cpp $(SRCDIR)/internalTime.cpp $(SRCDIR)/ObsData.cpp $(SRCDIR)/navigation.cpp $(SRCDIR)/triple.cpp $(SRCDIR)/GTEC.cpp $(SRCDIR)/GTEC.cpp
TESTSDIR = tests
TESTSSRC = $(TESTSDIR)/test_modip.cpp
TESTS = test_modip
CTSTFLAGS = -std=c++11 -I$(SRCDIR)

#------------------------------------------------------------------------------

$(PROGRAM): $(OBJS)
	$(CC) $(OBJS) -o $(PROGRAM) $(CFLAGS) $(LDFLAGS)

ptr_pair.o:  $(SRCDIR)/ptr_pair.cpp
	$(CC) -c $(SRCDIR)/ptr_pair.cpp $(CFLAGS)

int_pair.o:  $(SRCDIR)/int_pair.cpp
	$(CC) -c $(SRCDIR)/int_pair.cpp $(CFLAGS)

inout.o:  $(SRCDIR)/inout.cpp
	$(CC) -c $(SRCDIR)/inout.cpp $(CFLAGS)
	
internalTime.o:  $(SRCDIR)/internalTime.cpp
	$(CC) -c $(SRCDIR)/internalTime.cpp $(CFLAGS)

navigation.o:  $(SRCDIR)/navigation.cpp
	$(CC) -c $(SRCDIR)/navigation.cpp $(CFLAGS)

ObsData.o:  $(SRCDIR)/ObsData.cpp
	$(CC) -c $(SRCDIR)/ObsData.cpp $(CFLAGS)

triple.o:  $(SRCDIR)/triple.cpp
	$(CC) -c $(SRCDIR)/triple.cpp $(CFLAGS)

igrf.o:  $(SRCDIR)/igrf.cpp
	$(CC) -c $(SRCDIR)/igrf.cpp $(CFLAGS)
	
GTEC.o:  $(SRCDIR)/GTEC.cpp
	$(CC) -c $(SRCDIR)/GTEC.cpp $(CFLAGS)


.PHONY: tests
tests: $(TESTS)

test_modip: test_modip.o triple.o igrf.o
	$(CC) test_modip.o triple.o igrf.o -o test_modip 

test_modip.o: $(TESTSDIR)/test_modip.cpp
	$(CC) -c $(TESTSDIR)/test_modip.cpp $(CTSTFLAGS)


.PHONY: all
all: $(PROGRAM) tests

.PHONY: clean
clean:
	rm -f $(PROGRAM) $(TESTS)
	rm -f *.o
