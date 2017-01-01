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


#include "triple.hpp"

	void triple::dump(std::ostream& s)
	{
		s << X <<  " " << Y << " " << Z << "\n";
	};
