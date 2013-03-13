#ifndef COMMON_H
#define COMMON_H

#include <CVars/CVar.h>
#include <Eigen/Dense>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global Variables

const int               MAX_PYR_LEVELS = 5;


Eigen::Matrix4d         g_Tvr = Eigen::Matrix4d::Identity();    // robotic to vision transform
Eigen::Matrix4d         g_Trv = Eigen::Matrix4d::Identity();    // vision to robotic transform
bool                    g_bHaveDepth = false;                   // true if depth maps are given to us by the camera.



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Global CVARS

bool&                                   g_bDisparityMaps  = CVarUtils::CreateCVar( "Cam.DisparityMaps", false, "True if camera gives disparity maps instead of depth maps." );
bool&                                   g_bAligned  = CVarUtils::CreateCVar( "Cam.Aligned", false, "True if color and depth cameras are aligned." );
float&                                  g_fDepthScale   = CVarUtils::CreateCVar( "Cam.DepthScale", 1.0f, "Scale of depth information provided by camera. Depth should be meters." );
bool&                                   g_bBiFilterThumbs = CVarUtils::CreateCVar( "Keys.Thumbs.FilterOn", true,
                                                                                   "True if cross-bilateral filter is to be applied to the thumbnail depth map." );
unsigned int&                           g_nThumbFiltSize = CVarUtils::CreateCVar( "Keys.Thumbs.FilterSize", 5u, "Filter size." );
double&                                 g_dThumbFiltS   = CVarUtils::CreateCVar( "Keys.Thumbs.FilterSpatial", 1e-3, "Filter spatial component." );
double&                                 g_dThumbFiltD   = CVarUtils::CreateCVar( "Keys.Thumbs.FilterDepth", 1e-3, "Filter depth component." );
double&                                 g_dThumbFiltC   = CVarUtils::CreateCVar( "Keys.Thumbs.FilterColor", 1e-3, "Filter color component." );
Eigen::Matrix<int,1,Eigen::Dynamic>&    g_vPyrMaxIters  = CVarUtils::CreateCVar( "Tracker.PyrMaxIters", Eigen::Matrix<int,1,Eigen::Dynamic>(),
                                                                                 "Maximum number of iterations per pyramid level." );
Eigen::Matrix<int,1,Eigen::Dynamic>&    g_vPyrFullMask  = CVarUtils::CreateCVar( "Tracker.PyrFullMask", Eigen::Matrix<int,1,Eigen::Dynamic>(),
                                                                                 "Set 1 for full estimate, 0 for rotation only estimates." );
Eigen::Vector6d&                        g_vMotionModel  = CVarUtils::CreateCVar( "Tracker.MotionModel", Eigen::Vector6d(),
                                                                                 "Motion model used to discard bad estimates." );
unsigned int&                           g_nPoseDisplay  = CVarUtils::CreateCVar( "Gui.PoseDisplay", 5u, "Number of axis to draw for poses." );



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void InitPermutationMatrices()
{
    Eigen::Matrix3d RDFvision;
    RDFvision << 1, 0, 0,
                 0, 1, 0,
                 0, 0, 1;
    Eigen::Matrix3d RDFrobot;
    RDFrobot << 0, 1, 0,
                0, 0, 1,
                1, 0, 0;
    g_Tvr.block < 3, 3 > (0, 0) = RDFvision.transpose( ) * RDFrobot;
    g_Trv.block < 3, 3 > (0, 0) = RDFrobot.transpose( ) * RDFvision;
}

#endif // COMMON_H
