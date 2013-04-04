#ifndef _DENSE_BACK_END_CONFIG_H_
#define _DENSE_BACK_END_CONFIG_H_

#include <CVars/CVar.h>
#include <Utils/CVarHelpers.h>

class DenseBackEndConfig
{
public:
    DenseBackEndConfig() :

        // MOTION MODEL OPTIONS
        g_bConstantVelocityMotionModel( CVarUtils::CreateCVar<>( "BackEnd.UseConstantVelocityMotionModel", false, "Use constant velocity motion model." ) ),
        g_bAlwaysUseLastKeyframe( CVarUtils::CreateCVar<>( "BackEnd.AlwaysUseLastKeyframe", true, "Do not search for closest keyframe, but rather choose last keyframe used." ) ),

        // DEBUG OPTIONS
        g_nErrorLevel( CVarUtils::CreateCVar<>( "BackEnd.Debug.ErrorLevel", 0, "Verbosity level for printing errors." ) ),
        g_nDebugCERES( CVarUtils::CreateCVar<>( "BackEnd.Debug.CERES", 1u, "Print error level"))
    {}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    bool&                                   g_bConstantVelocityMotionModel;
    bool&                                   g_bAlwaysUseLastKeyframe;

    int&                                    g_nErrorLevel;
    unsigned int&                           g_nDebugCERES;
};

#endif
