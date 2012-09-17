#ifndef _CVAR_HELPERS_
#define _CVAR_HELPERS_

#include <CVars/CVarVectorIO.h>

#include <Eigen/Core>

////////////////////////////////////////////////////////////////////////////
// Overloading Eigen for CVars
namespace CVarUtils
{
    ////////////////////////////////////////////////////////////////////////////
    inline std::ostream& operator<<( std::ostream& Stream, Eigen::Matrix<int,1,Eigen::Dynamic>& Mat )
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
    inline std::istream& operator>>( std::istream& Stream, Eigen::Matrix<int,1,Eigen::Dynamic>& Mat )
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


////////////////////////////////////////////////////////////////////////////
// Overloading GUI
std::ostream& operator<< (std::ostream& os, const Eigen::Vector6d& v)
{
    os << "( " << std::fixed << std::setprecision(2) << std::showpos << v(0) << ", " << v(1) << ", " << v(2) << ", " << v(3) << ", " << v(4) << ", " << v(5) << " )";
    return os;
}

////////////////////////////////////////////////////////////////////////////
std::istream& operator>> (std::istream& is, Eigen::Vector6d& v)
{
  is >> v(0);
  is >> v(1);
  is >> v(2);
  is >> v(3);
  is >> v(4);
  is >> v(5);
  return is;
}







#endif
