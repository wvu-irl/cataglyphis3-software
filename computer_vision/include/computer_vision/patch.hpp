#include <iostream>
#include <string>
#include <sstream>

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
}

#endif //PATCH_HPP