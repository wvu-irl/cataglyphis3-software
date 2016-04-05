#include <iostream>
#include <string>
#include <sstream>
#include <ctime>

#ifndef PATCH_HPP
#define PATCH_HPP

namespace patch  
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }

	const std::string currentDateTime() 
	{
	    time_t     now = time(0);
	    struct tm  tstruct;
	    char       buf[80];
	    tstruct = *localtime(&now);
	    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
	    return buf;
	}
}

#endif //PATCH_HPP