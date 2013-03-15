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
    g_nErrorLevel( CVarUtils::CreateCVar<>( "debug.ErrorLevel", 0, "Verbosity level for printing errors." ) ),
    g_nDebugCERES( CVarUtils::CreateCVar<>( "debug.CERES", 1u, "Print error level"))
    {}

    int&                    g_nErrorLevel;
    unsigned int&           g_nDebugCERES;
};

#endif
