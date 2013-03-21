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
        g_bConstantVelocityMotionModel( CVarUtils::CreateCVar<>( "tracker.UseConstantVelocityMotionModel", false, "Use constant velocity motion model." ) ),

        // LOOP CLOSURE OPTIONS
        g_nLoopClosureMargin( CVarUtils::CreateCVar<>( "tracker.LoopClosureMargin", 20u, "Number of frames between current frame which will not be taken in consideration for loop closure." ) ),
        g_dLoopClosureThreshold( CVarUtils::CreateCVar<>( "tracker.LoopClosureThreshold", 5.0, "Maximum RMSE in order to flag a loop closure." ) ),

        // DEBUG OPTIONS
        g_nErrorLevel( CVarUtils::CreateCVar<>( "debug.ErrorLevel", 0, "Verbosity level for printing errors." ) ),
        g_nDebugESM( CVarUtils::CreateCVar<>( "debug.ESM", 1u, "Print error level")),
        g_nDebugCERES( CVarUtils::CreateCVar<>( "debug.CERES", 1u, "Print error level"))
    {}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Eigen::Matrix<int,1,Eigen::Dynamic>&    g_vPyrMaxIters;
    Eigen::Matrix<int,1,Eigen::Dynamic>&    g_vPyrFullMask;
    bool&                                   g_bConstantVelocityMotionModel;
    unsigned int&                           g_nLoopClosureMargin;
    double&                                 g_dLoopClosureThreshold;
    int&                                    g_nErrorLevel;
    unsigned int&                           g_nDebugESM;
    unsigned int&                           g_nDebugCERES;
};

#endif
