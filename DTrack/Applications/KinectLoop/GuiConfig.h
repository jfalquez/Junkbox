#ifndef _GUI_CONFIG_H_
#define _GUI_CONFIG_H_

#include <CVars/CVar.h>

class GuiConfig
{
public:
    GuiConfig() :

        // GLGRID OPTIONS
        g_nNumGridLines( CVarUtils::CreateCVar<>( "ui.NumGridLines", 10u, "Number of grid divisions." ) ),

        // GLPATH OPTIONS
        g_nNumPosesToShow( CVarUtils::CreateCVar<>( "ui.NumPosesToShow", 1u, "Number of poses to display on GUI. Setting to 0 displays all poses." ) ),

        // GLMAP OPTIONS
        g_nNumVBOsToShow( CVarUtils::CreateCVar<>( "ui.NumVBOsToShow", 0u, "Number of VBOs to render on GUI. Setting to 0 displays all VBOs." ) ),
        g_bShowHiResMap( CVarUtils::CreateCVar<>( "ui.ShowHiResMap", false, "True if hi resolution images are used for rendering." ) )
    {}

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    unsigned int&           g_nNumGridLines;
    unsigned int&           g_nNumPosesToShow;
    unsigned int&           g_nNumVBOsToShow;
    bool&                   g_bShowHiResMap;
};

#endif
