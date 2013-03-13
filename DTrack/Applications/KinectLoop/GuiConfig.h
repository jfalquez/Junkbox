#ifndef _GUI_CONFIG_H_
#define _GUI_CONFIG_H_

#include <CVars/CVar.h>

class GuiConfig
{
public:
    GuiConfig() :
   // m_nTimerDepth( CVarUtils::CreateCVar<>( "gui.TimerDepth", 2, "Timer depth for display" ) ),
   //m_nTimerWindowSize( CVarUtils::CreateCVar<>( "gui.TimerWindowSize", 40, "Timer time window size" ) ),
   // m_dPathOrientation( CVarUtils::CreateCVar<>( "gui.PathOrientation", Eigen::Vector3d( Eigen::Vector3d::Zero() ), "SLAM path base orientation on GUI." ) ),
    m_uNumPosesToShow( CVarUtils::CreateCVar<>( "gui.NumPosesToShow", 20u, "Number of poses to display on GUI. Setting to 0 displays all poses." ) ),
    m_uNumGridLines( CVarUtils::CreateCVar<>( "gui.NumGridLines", 10u, "Number of grid divisions." ) ),
   // m_sGroundTruthFile( CVarUtils::CreateCVar<>( "gui.GroundTruthFile", std::string(""), "File containing the ground truth poses (relative poses)." ) ),
    m_nStartFrame( CVarUtils::CreateCVar<>( "gui.StartFrame", 0u, "Starting image frame for FileReader." ) )
    {}
    
   // int&                  m_nTimerDepth;
   // int&                  m_nTimerWindowSize;
   // Eigen::Vector3d&      m_dPathOrientation;
    unsigned int&         m_uNumPosesToShow;
    unsigned int&         m_uNumGridLines;
   // std::string&	      m_sGroundTruthFile;
    unsigned int&	      m_nStartFrame;
};

#endif
