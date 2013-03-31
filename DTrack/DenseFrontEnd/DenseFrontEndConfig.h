#ifndef _DENSE_FRONT_END_CONFIG_H_
#define _DENSE_FRONT_END_CONFIG_H_

#include <CVars/CVar.h>
#include <Utils/CVarHelpers.h>

const int   MAX_PYR_LEVELS = 5;

class DenseFrontEndConfig
{
public:
    DenseFrontEndConfig() :

        // MOTION MODEL OPTIONS
        g_bConstantVelocityMotionModel( CVarUtils::CreateCVar<>( "tracker.UseConstantVelocityMotionModel", false, "Use constant velocity motion model." ) ),
        g_bAlwaysUseLastKeyframe( CVarUtils::CreateCVar<>( "tracker.AlwaysUseLastKeyframe", false, "Do not search for closest keyframe, but rather choose last keyframe used." ) ),

        // LOCALIZATION OPTIONS
        g_vPyrMaxIters( CVarUtils::CreateCVar( "tracker.ESM.PyrMaxIters", Eigen::Matrix<int,1,Eigen::Dynamic>(),
                                                "Maximum number of iterations per pyramid level." ) ),
        g_vPyrFullMask( CVarUtils::CreateCVar( "tracker.ESM.PyrFullMask", Eigen::Matrix<int,1,Eigen::Dynamic>(),
                                                "Set 1 for full estimate, 0 for rotation only estimates." ) ),

        // KEYFRAME OPTIONS
        g_fCloseKeyframeNorm( CVarUtils::CreateCVar<>( "tracker.CloseKeyframeNorm", 3.0f, "Maximum norm for keyframes to be selected in a neighborhood." ) ),
        g_fKeyframePtsThreshold( CVarUtils::CreateCVar<>( "tracker.KeyframePtsThreshold", 0.8f, "Minimum percentage of points before a new keyframe is added to the map." ) ),

        // LOOP CLOSURE OPTIONS
        g_fLoopClosureRadius( CVarUtils::CreateCVar<>( "tracker.LoopClosure.Radius", 5.0f, "Norm radius for frames to be taken in consideration for loop closure." ) ),
        g_nLoopClosureMargin( CVarUtils::CreateCVar<>( "tracker.LoopClosure.FrameMargin", 20u, "Number of frames between current frame which will not be taken in consideration for loop closure." ) ),
        g_nLoopClosureSAD( CVarUtils::CreateCVar<>( "tracker.LoopClosure.SAD", 5u, "Maximum SAD score for loop closure candidates. This value is multiplied by number of pixels in thumbnail." ) ),
        g_dLoopClosureThreshold( CVarUtils::CreateCVar<>( "tracker.LoopClosure.MaximumRMSE", 8.0, "Maximum RMSE in order to accept a loop closure." ) ),

        // DEBUG OPTIONS
        g_nErrorLevel( CVarUtils::CreateCVar<>( "debug.ErrorLevel", 0, "Verbosity level for printing errors." ) ),
        g_nDebugESM( CVarUtils::CreateCVar<>( "debug.ESM", 1u, "Print error level")),
        g_nDebugCERES( CVarUtils::CreateCVar<>( "debug.CERES", 1u, "Print error level"))
    {}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool&                                   g_bConstantVelocityMotionModel;
    bool&                                   g_bAlwaysUseLastKeyframe;
    Eigen::Matrix<int,1,Eigen::Dynamic>&    g_vPyrMaxIters;
    Eigen::Matrix<int,1,Eigen::Dynamic>&    g_vPyrFullMask;
    float&                                  g_fCloseKeyframeNorm;
    float&                                  g_fKeyframePtsThreshold;
    float&                                  g_fLoopClosureRadius;
    unsigned int&                           g_nLoopClosureMargin;
    unsigned int&                           g_nLoopClosureSAD;
    double&                                 g_dLoopClosureThreshold;
    int&                                    g_nErrorLevel;
    unsigned int&                           g_nDebugESM;
    unsigned int&                           g_nDebugCERES;
};

#endif
