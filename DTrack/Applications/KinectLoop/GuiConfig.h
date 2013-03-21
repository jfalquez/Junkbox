#ifndef _GUI_CONFIG_H_
#define _GUI_CONFIG_H_

#include <CVars/CVar.h>

class GuiConfig
{
public:
    GuiConfig() :
    g_nNumPosesToShow( CVarUtils::CreateCVar<>( "ui.NumPosesToShow", 20u, "Number of poses to display on GUI. Setting to 0 displays all poses." ) ),
    g_nNumGridLines( CVarUtils::CreateCVar<>( "ui.NumGridLines", 10u, "Number of grid divisions." ) ),
    g_nStartFrame( CVarUtils::CreateCVar<>( "ui.StartFrame", 0u, "Starting image frame for FileReader." ) )
    {}

    unsigned int&         g_nNumPosesToShow;
    unsigned int&         g_nNumGridLines;
    unsigned int&	      g_nStartFrame;
};

#endif
