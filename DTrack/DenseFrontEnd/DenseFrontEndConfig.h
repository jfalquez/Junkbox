#ifndef _DENSE_FRONT_END_CONFIG_H_
#define _DENSE_FRONT_END_CONFIG_H_

#include <CVars/CVar.h>
#include <Utils/CVarHelpers.h>

class DenseFrontEndConfig
{
public:
    DenseFrontEndConfig() :
    // FRONT END - GENERAL OPTIONS
    g_nErrorLevel( CVarUtils::CreateCVar<>( "fe.ErrorLevel", 0, "Verbosity level for printing errors." ) )
    {}

    int&                  g_nErrorLevel;
};

#endif
