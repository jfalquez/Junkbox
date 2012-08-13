#ifndef _CVAR_HELPERS_
#define _CVAR_HELPERS_

#include <CVars/CVarVectorIO.h>

#include <Eigen/Core>

////////////////////////////////////////////////////////////////////////////
// Overloading Eigen for CVars
namespace CVarUtils
{
    inline std::ostream& operator<<( std::ostream& Stream, Eigen::Vector6d& Mat )
    {
        unsigned int nRows = Mat.rows();
        unsigned int nCols = Mat.cols();

        Stream << "[ ";

        for( unsigned int ii = 0; ii < nRows-1; ii++ ) {
            for( unsigned int jj = 0; jj < nCols-1; jj++ ) {
                Stream << Mat(ii, jj);
                Stream << ", ";
            }
            Stream << Mat(ii, nCols-1);
            Stream << "; ";
        }
        for( unsigned int jj = 0; jj < nCols-1; jj++ ) {
            Stream << Mat(nRows-1, jj);
            Stream << ", ";
        }
        Stream << Mat(nRows-1, nCols-1);
        Stream << " ]";

        return Stream;
    }

    ////////////////////////////////////////////////////////////////////////////
    inline std::istream& operator>>( std::istream& Stream, Eigen::Vector6d& Mat )
    {

        unsigned int nRows = Mat.rows();
        unsigned int nCols = Mat.cols();
        char str[256];

        Stream.getline(str, 255, '[');
        if( Stream.gcount() > 1 ) {
            return Stream;
        }
        for( unsigned int ii = 0; ii < nRows-1; ii++ ) {
            for( unsigned int jj = 0; jj < nCols-1; jj++ ) {
                Stream.getline(str, 255, ',');
                Mat(ii, jj) = std::strtod(str, NULL);
            }
            Stream.getline(str, 255, ';');
            Mat(ii, nCols-1) = std::strtod(str, NULL);
        }
        for( unsigned int jj = 0; jj < nCols-1; jj++ ) {
            Stream.getline(str, 255, ',');
            Mat(nRows-1, jj) = std::strtod(str, NULL);
        }
        Stream.getline(str, 255, ']');
        Mat(nRows-1, nCols-1) = std::strtod(str, NULL);
        return Stream;
    }

}

#endif