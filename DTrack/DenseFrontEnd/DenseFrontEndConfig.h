#ifndef _DENSE_FRONT_END_CONFIG_H_
#define _DENSE_FRONT_END_CONFIG_H_

#include <CVars/CVar.h>
#include <Utils/CVarHelpers.h>

const int   MAX_PYR_LEVELS = 5;

class DenseFrontEndConfig
{
public:
    DenseFrontEndConfig() :
        // FRONT END - GENERAL OPTIONS
        g_vPyrMaxIters( CVarUtils::CreateCVar( "tracker.PyrMaxIters", Eigen::Matrix<int,1,Eigen::Dynamic>(),
                                                "Maximum number of iterations per pyramid level." ) ),
        g_vPyrFullMask( CVarUtils::CreateCVar( "tracker.PyrFullMask", Eigen::Matrix<int,1,Eigen::Dynamic>(),
                                                "Set 1 for full estimate, 0 for rotation only estimates." ) ),

        // DEBUG OPTIONS
        g_nErrorLevel( CVarUtils::CreateCVar<>( "debug.ErrorLevel", 0, "Verbosity level for printing errors." ) ),
        g_nDebugCERES( CVarUtils::CreateCVar<>( "debug.CERES", 1u, "Print error level"))
    {}

    Eigen::Matrix<int,1,Eigen::Dynamic>&    g_vPyrMaxIters;
    Eigen::Matrix<int,1,Eigen::Dynamic>&    g_vPyrFullMask;
    int&                                    g_nErrorLevel;
    unsigned int&                           g_nDebugCERES;
};

#endif
