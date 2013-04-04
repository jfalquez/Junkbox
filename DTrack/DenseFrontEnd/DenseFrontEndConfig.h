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
        g_bConstantVelocityMotionModel( CVarUtils::CreateCVar<>( "FrontEnd.UseConstantVelocityMotionModel", false, "Use constant velocity motion model." ) ),
        g_bAlwaysUseLastKeyframe( CVarUtils::CreateCVar<>( "FrontEnd.AlwaysUseLastKeyframe", false, "Do not search for closest keyframe, but rather choose last keyframe used." ) ),

        // LOCALIZATION OPTIONS
        g_vPyrMaxIters( CVarUtils::CreateCVar( "FrontEnd.GN.PyrMaxIters", Eigen::Matrix<int,1,Eigen::Dynamic>(),
                                                "Maximum number of iterations per pyramid level." ) ),
        g_vPyrFullMask( CVarUtils::CreateCVar( "FrontEnd.GN.PyrFullMask", Eigen::Matrix<int,1,Eigen::Dynamic>(),
                                                "Set 1 for full estimate, 0 for rotation only estimates." ) ),

        // KEYFRAME OPTIONS
        g_fCloseKeyframeNorm( CVarUtils::CreateCVar<>( "FrontEnd.CloseKeyframeNorm", 3.0f, "Maximum norm for keyframes to be selected in a neighborhood." ) ),
        g_fKeyframePtsThreshold( CVarUtils::CreateCVar<>( "FrontEnd.KeyframePtsThreshold", 0.75f, "Minimum percentage of points before a new keyframe is added to the map." ) ),

        // LOOP CLOSURE OPTIONS
        g_fLoopClosureRadius( CVarUtils::CreateCVar<>( "FrontEnd.LoopClosure.Radius", 5.0f, "Norm radius for frames to be taken in consideration for loop closure." ) ),
        g_nLoopClosureMargin( CVarUtils::CreateCVar<>( "FrontEnd.LoopClosure.FrameMargin", 20u, "Number of frames between current frame which will not be taken in consideration for loop closure." ) ),
        g_nLoopClosureSAD( CVarUtils::CreateCVar<>( "FrontEnd.LoopClosure.SAD", 5u, "Maximum SAD score for loop closure candidates. This value is multiplied by number of pixels in thumbnail." ) ),
        g_dLoopClosureThreshold( CVarUtils::CreateCVar<>( "FrontEnd.LoopClosure.MaximumRMSE", 8.0, "Maximum RMSE in order to accept a loop closure." ) ),

        // DEBUG OPTIONS
        g_nErrorLevel( CVarUtils::CreateCVar<>( "FrontEnd.Debug.ErrorLevel", 0, "Verbosity level for printing errors." ) ),
        g_nDebugGN( CVarUtils::CreateCVar<>( "FrontEnd.Debug.GN", 1u, "Print error level"))
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
    unsigned int&                           g_nDebugGN;
};

#endif
