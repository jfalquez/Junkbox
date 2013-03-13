#ifndef __GUI_H_
#define	__GUI_H_

#include <iostream>
#include <string>
#include <boost/bind.hpp>
#include <opencv/cv.h>
#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>
#include <Mvlpp/Mvl.h>
#include <mvl/camera/camera_convert.h>
#include <FrontEndGui/HelpView.h>
#include <FrontEndGui/TimerView.h>
#include <FrontEndGui/AnalyticsView.h>
#include <FrontEndGui/StereoView.h>
#include <FrontEndGui/PatchView.h>
#include <FrontEndGui/LandmarkInfoView.h>
#include <FrontEndGui/GLMeasurements.h>
#include <FrontEndGui/GLCorrespondences.h>
#include <FrontEndGui/GLPath.h>
#include <FrontEndGui/GLKeypoints.h>
#include <FrontEndGui/GuiConfig.h>
#include <Utils/PatchUtils.h>
#include <Utils/VehicleConfig.h>
#include <Communication/Vo2Gui.h>

#include "GuiConfig.h"


GuiConfig g_GuiConfig;

enum GuiState { PLAYING, STEPPING, PAUSED, RESETTING, RESET_COMPLETE, GETSTATE, QUIT };

class Gui
{

    public:


        Gui();
        Gui(const std::string sWindowName, const int nWidth, const int nHeight);

        void Init();
        void InitReset();
        void Run();
        void CopyMapChanges( Map& rMap );
        void SetVehicleConfig( VehicleConfig& rVC );

        // data updates
        void UpdateFrames(cv::Mat& lframe, cv::Mat& rframe);

        void UpdateKeypoints(
                const std::vector<cv::KeyPoint>& kpts_left,
                const std::vector<cv::KeyPoint>& kpts_right
                );

        void UpdateTimer(
                int                                      nTimeWindowSize,
                std::vector< std::string>                vNames,
                std::vector< std::pair<double,double> >  vTimes
                );

        void UpdateAnalytics( std::map< std::string, double >& mData );

        // these updates will be replaced my CopyMapChanges
        void UpdateActiveLandmarks( const std::vector<Landmark>& vLandmarks );

        void SetState( GuiState s )
        {
            state = s;
        }

        void MouseMotion( int x, int y );

    private:

        unsigned char* _LoadMeasurementImagePatch( Measurement&   z,
                                                   unsigned int   cam,
                                                   unsigned char* data );

        void _RegisterKeyboardCallbacks();

        void _SaveDataToFile();

        void _UpdateCameraView();

        void _RIGHT_ARROW()
        {
            state = STEPPING;
        }

        void _SPACE_BAR()
        {
            state = (state == PAUSED)?PLAYING:PAUSED;
        }

        void _CTRL_R()
        {
            state = RESETTING;
            while( state != RESET_COMPLETE ){
                usleep(10000);
            }
            InitReset(); // not called from elsewhere
            state = PAUSED;
        }

         void _CTRL_S()
        {
            _SaveDataToFile();
        }

        void _KEYBOARD_K()
        {
             m_glCurKeypointsLeft.ToggleShow();
             m_glCurKeypointsRight.ToggleShow();
             if( !m_bFirstFrame ) {
                 m_glPrevKeypointsLeft.ToggleShow();
                 m_glPrevKeypointsRight.ToggleShow();
             }
        }

        void _KEYBOARD_P()
        {
            if( !m_bFirstFrame ){
                m_PrevImageLeft.ToggleShow();
                m_PrevImageRight.ToggleShow();
                m_glMeasurements.ToggleShowPreviousMeasurements();
            }
        }

        void _KEYBOARD_C()
        {
            m_glMeasurements.ToggleShowCorrespondences();
        }

        void _KEYBOARD_T()
        {
            m_glMeasurements.ToggleShowTrajectories();
        }

        void _KEYBOARD_I()
        {
            m_LandmarkInfoView.ToggleShow();
        }

        void _CTRL_P()
        {
            m_PatchView.ToggleShow();
        }

        void _KEYBOARD_H()
        {
            m_HelpView.ToggleShow();
        }

    public:
        volatile GuiState             state;

    private:

        bool                         m_bMapDirty;
        bool                         m_bFirstFrame;

        Map*                         m_pChangesBufferMap;   // re-allocated on reset. for now.
        Map*                         m_pRenderMap;          // re-allocated on reset. for now.

        int                          m_nWindowWidth;
        int                          m_nWindowHeight;
        int                          m_nImageWidth;
        int                          m_nImageHeight;
        std::string                  m_sWindowName;
        pangolin::View               m_View3d;
        HelpView                     m_HelpView;
        StereoView                   m_StereoView;
        TimerView                    m_TimerView;
        AnalyticsView                m_AnalyticsView;
        PatchView                    m_PatchView;
        LandmarkInfoView             m_LandmarkInfoView;

        pangolin::OpenGlRenderState  m_glRender3d;
        pangolin::OpenGlRenderState  m_glRender2d;

        // GL Objects
        SceneGraph::GLSceneGraph     m_glGraph2d;
        SceneGraph::GLSceneGraph     m_glGraph3d;
        SceneGraph::GLGrid           m_glGrid;
        SceneGraph::GLSceneGraph     m_glCurLeftImageGraph;
        SceneGraph::GLSceneGraph     m_glCurRightImageGraph;
        SceneGraph::GLSceneGraph     m_glPrevLeftImageGraph;
        SceneGraph::GLSceneGraph     m_glPrevRightImageGraph;
        // our own SceneGraph objects
        GLPath                       m_glPath;
        GLKeypoints                  m_glCurKeypointsLeft;
        GLKeypoints                  m_glCurKeypointsRight;
        GLKeypoints                  m_glPrevKeypointsLeft;
        GLKeypoints                  m_glPrevKeypointsRight;
        GLMeasurements               m_glMeasurements;

        SceneGraph::GLText           m_glText;

        // Stereo pair
        SceneGraph::ImageView        m_CurImageLeft;
        SceneGraph::ImageView        m_CurImageRight;
        SceneGraph::ImageView        m_PrevImageLeft;
        SceneGraph::ImageView        m_PrevImageRight;

        std::map<LandmarkId, Landmark> m_mActiveLandmarks;
        boost::mutex                   m_Mutex;
        VehicleConfig                  m_VehicleCfg;

};

/////////////////////////////////////////////////////////////////////////
// Implementation
/////////////////////////////////////////////////////////////////////////
Gui::Gui()
{
     Gui( "GWU SLAM Engine", 640, 480 );
}

/////////////////////////////////////////////////////////////////////////
Gui::Gui(const std::string sWindowName, const int nWidth, const int nHeight)
{
    m_sWindowName        = sWindowName;
    m_nWindowWidth       = nWidth;
    m_nWindowHeight      = nHeight;
    m_nImageWidth        = 0;
    m_nImageHeight       = 0;
    m_bMapDirty          = false;
    m_pChangesBufferMap  = NULL;
    m_pRenderMap         = NULL;
    state                = PLAYING;

}

/////////////////////////////////////////////////////////////////////////
void Gui::Init()
{
    // Create OpenGL window in single line thanks to GLUT
    pangolin::CreateGlutWindowAndBind(m_sWindowName,m_nWindowWidth,m_nWindowHeight );

    // Register keyboard callbacks
    _RegisterKeyboardCallbacks();

    // Add views to window
    pangolin::DisplayBase().AddDisplay( m_View3d );
    pangolin::DisplayBase().AddDisplay( m_StereoView );
    pangolin::DisplayBase().AddDisplay( m_TimerView );
    pangolin::DisplayBase().AddDisplay( m_AnalyticsView );
    pangolin::DisplayBase().AddDisplay( m_PatchView );
    pangolin::DisplayBase().AddDisplay( m_LandmarkInfoView );
    pangolin::DisplayBase().AddDisplay( m_HelpView );

    // Register to receive mouse motion callbacks from StereoView
    m_StereoView.RegisterPassiveMouseMotionCallback( boost::bind( &Gui::MouseMotion, this, _1, _2 ) );

    // Set up SceneGraphs
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    m_glGraph3d.AddChild( &m_glGrid );
    m_glGraph3d.AddChild( &m_glPath );

    m_glCurLeftImageGraph.AddChild( &m_glCurKeypointsLeft );
    m_glCurRightImageGraph.AddChild( &m_glCurKeypointsRight );
    m_glPrevLeftImageGraph.AddChild( &m_glPrevKeypointsLeft );
    m_glPrevRightImageGraph.AddChild( &m_glPrevKeypointsRight );

    // Define Camera Render Object (for view / scene browsing)
    m_glRender3d.SetProjectionMatrix( pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,5000) );
    m_glRender3d.SetModelViewMatrix( pangolin::ModelViewLookAt(0,-2,-4, 0,1,0, pangolin::AxisNegZ) );
    m_glRender2d.SetModelViewMatrix( pangolin::IdentityMatrix() );
    m_glRender2d.SetProjectionMatrix( pangolin::ProjectionMatrixOrthographic(0,640,0,480,0,1000) );

    // Set view for drawing the map and vehicle path, bounds: (bottom,top,left,right)
    m_View3d.SetBounds( 0.0, 0.6, 0.0, 1.0, -640.0f/480.0f );
    m_View3d.SetHandler( new SceneGraph::HandlerSceneGraph( m_glGraph3d, m_glRender3d, pangolin::AxisNegZ) );

    // get GLText, add GLText to 2dSG, put 2dSG into pangolin ( add it to a ViewPort, setup matrices)
    m_View3d.SetDrawFunction( SceneGraph::ActivateDrawFunctor3d2d( m_glGraph3d, m_glRender3d, m_glGraph2d, m_glRender2d) );

    // Set view for displaying stereo images and features
    m_StereoView.SetBounds( 0, 1.0, 0.0, 0.7, 512.0f/384.0f );
    m_StereoView.SetLock( pangolin::LockLeft, pangolin::LockTop );
    m_StereoView.AddDisplay( m_CurImageLeft );
    m_StereoView.AddDisplay( m_CurImageRight );
    m_StereoView.AddDisplay( m_PrevImageLeft );
    m_StereoView.AddDisplay( m_PrevImageRight );
    m_StereoView.SetPassThroughView( m_View3d );
    m_CurImageLeft.SetBounds( 0.50, 1.00, 0.00, 0.50 );
    m_CurImageRight.SetBounds( 0.50, 1.00, 0.50, 1.00 );
    m_PrevImageLeft.SetBounds( 0.00, 0.50, 0.00, 0.50 );
    m_PrevImageRight.SetBounds( 0.00, 0.50, 0.50, 1.00 );
    m_CurImageLeft.SetDrawFunction( SceneGraph::ActivateDrawFunctor( m_glCurLeftImageGraph, m_glRender2d) );
    m_CurImageRight.SetDrawFunction( SceneGraph::ActivateDrawFunctor( m_glCurRightImageGraph, m_glRender2d) );
    m_PrevImageLeft.SetDrawFunction( SceneGraph::ActivateDrawFunctor( m_glPrevLeftImageGraph, m_glRender2d) );
    m_PrevImageRight.SetDrawFunction( SceneGraph::ActivateDrawFunctor( m_glPrevRightImageGraph, m_glRender2d) );
    m_StereoView.SetDrawFunction( SceneGraph::ActivateDrawFunctor ( m_glMeasurements,  m_glRender2d ) );

    m_TimerView.SetBounds( 0.75, 1.0, 0.7, 1.0 );
    m_AnalyticsView.SetBounds( 0.55, 0.75, 0.7, 1.0 );
    m_PatchView.SetBounds( 0.35, 0.55, 0.7, 0.8, 2.0f/3.0f);
    m_LandmarkInfoView.SetBounds( 0.0, 0.55, 0.0, 0.7 );
    m_HelpView.SetBounds( 0.0, 0.4, 0.7, 1.0 );

    InitReset();
}

///////////////////////////////////////////////////////////////////////////////
void Gui::InitReset()
{
    m_Mutex.lock();
    if( m_pChangesBufferMap ){ delete m_pChangesBufferMap; }
    if( m_pRenderMap )       { delete m_pRenderMap; }

    m_pChangesBufferMap = new Map; // this receives data from the front end
    m_pRenderMap        = new Map; // this is used by GLObjects
    m_bMapDirty         = false;
    m_Mutex.unlock();

    // Init path
    m_glPath.InitReset( m_pRenderMap );
    m_LandmarkInfoView.InitReset( m_pRenderMap );
    //m_glPath.SetNumPosesToShow( g_GuiConfig.m_uNumPosesToShow );
    // Set view for displaying the timer
    m_TimerView.InitReset();
    m_AnalyticsView.InitReset();
    m_PatchView.InitReset();
    m_glCurKeypointsLeft.InitReset( m_nImageWidth, m_nImageHeight );
    m_glCurKeypointsRight.InitReset( m_nImageWidth, m_nImageHeight );
    m_glMeasurements.InitReset( m_nImageWidth, m_nImageHeight, m_pRenderMap );

    m_glGrid.SetNumLines( g_GuiConfig.m_uNumGridLines);
    m_glGrid.SetLineSpacing( 10.0 );

    // hide the previous images
    m_PrevImageLeft.Show( false );
    m_PrevImageRight.Show( false );
    m_glMeasurements.HidePreviousMeasurements();

    // hide keypoints by default
    m_glCurKeypointsLeft.Show( false );
    m_glCurKeypointsRight.Show( false );
    m_glPrevKeypointsLeft.Show( false );
    m_glPrevKeypointsRight.Show( false );

    // hide landmark info by default
    m_LandmarkInfoView.Show( false );

    // hide help info by default
    m_HelpView.Show( false );

    m_bFirstFrame = true;
}

///////////////////////////////////////////////////////////////////////////////
void Gui::Run()
{
    // Default hooks for exiting (Esc) and fullscreen (tab).
    while( 1 ) {

         if( m_bMapDirty ) {
            // the map has changed update it before rendering again
            m_Mutex.lock();
            m_pRenderMap->CopyMapChanges( *m_pChangesBufferMap );
            m_bMapDirty = false;
            m_Mutex.unlock();
         }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Swap frames and Process Events
        pangolin::FinishGlutFrame();

        usleep(1000000 / 120);
    }

    state = QUIT;
}

///////////////////////////////////////////////////////////////////////////////
void Gui::CopyMapChanges( Map& rMap )
{
    boost::mutex::scoped_lock lock(m_Mutex);
    m_pChangesBufferMap->CopyMapChanges( rMap );
    m_bMapDirty = true;
}

///////////////////////////////////////////////////////////////////////////////
void Gui::SetVehicleConfig( VehicleConfig& rVC )
{
    m_VehicleCfg = rVC;
    m_glMeasurements.SetVehicleCfg( rVC );
}

///////////////////////////////////////////////////////////////////////////////
void Gui::UpdateTimer(
        int  nTimeWindowSize,
        std::vector< std::string > vsNames,
        std::vector< std::pair<double,double> > vTimes
        )
{
    m_TimerView.Update( nTimeWindowSize, vsNames, vTimes );
}

///////////////////////////////////////////////////////////////////////////////
void Gui::UpdateFrames(cv::Mat& lframe, cv::Mat& rframe)
{
    int     cols   = lframe.cols;
    int     rows   = lframe.rows;

    if( m_nImageWidth == 0) {
        m_nImageWidth  = cols;
        m_nImageHeight = rows;
        m_glCurKeypointsLeft.InitReset( cols, rows );
        m_glCurKeypointsRight.InitReset( cols, rows );
        m_glPrevKeypointsLeft.InitReset( cols, rows );
        m_glPrevKeypointsRight.InitReset( cols, rows );
        m_glMeasurements.SetImageDims( cols, rows );
    } else {
        m_PrevImageLeft.SetImage( m_CurImageLeft.Data(), cols, rows, GL_LUMINANCE8, GL_LUMINANCE );
        m_PrevImageRight.SetImage( m_CurImageRight.Data() ,cols , rows, GL_LUMINANCE8, GL_LUMINANCE );

        // kind of hacky
        m_bFirstFrame = false;
    }

    m_CurImageRight.SetImage( rframe.data, cols, rows, GL_LUMINANCE8, GL_LUMINANCE );
    m_CurImageLeft.SetImage( lframe.data, cols, rows, GL_LUMINANCE8, GL_LUMINANCE );

}

///////////////////////////////////////////////////////////////////////////////
void Gui::UpdateActiveLandmarks( const std::vector<Landmark>& vLandmarks )
{
    //printf("updating landmarks");
    m_mActiveLandmarks.clear();
    for( size_t ii = 0; ii < vLandmarks.size(); ++ii ) {
        Landmark lm = vLandmarks[ii];
        lm.Id();
        m_mActiveLandmarks[ lm.Id() ] = lm;
    }
   // printf(" [done]\n");
}

///////////////////////////////////////////////////////////////////////////////
void Gui::UpdateKeypoints(
           const std::vector<cv::KeyPoint>& kpts_left,
           const std::vector<cv::KeyPoint>& kpts_right
          )
{
    m_glPrevKeypointsLeft.Update( m_glCurKeypointsLeft.Points() );

    m_glPrevKeypointsRight.Update( m_glCurKeypointsRight.Points() );

    m_glCurKeypointsLeft.Update( kpts_left );

    m_glCurKeypointsRight.Update( kpts_right );
}

///////////////////////////////////////////////////////////////////////////////
void Gui::UpdateAnalytics( std::map< std::string, double >& mData )
{
    m_AnalyticsView.Update( mData );
}

///////////////////////////////////////////////////////////////////////////////
void Gui::_RegisterKeyboardCallbacks()
{
    // step once
    pangolin::RegisterKeyPressCallback( pangolin::PANGO_SPECIAL + GLUT_KEY_RIGHT,
                                        boost::bind( &Gui::_RIGHT_ARROW, this) );

    // play / pause
    pangolin::RegisterKeyPressCallback( ' ',
                                        boost::bind( &Gui::_SPACE_BAR, this) );

    // reset app
    pangolin::RegisterKeyPressCallback( pangolin::PANGO_CTRL + 'r',
                                        boost::bind( &Gui::_CTRL_R, this) );
    // toggle patch matches
    pangolin::RegisterKeyPressCallback( pangolin::PANGO_CTRL + 'p',
                                        boost::bind( &Gui::_CTRL_P, this) );

    // save map data
    pangolin::RegisterKeyPressCallback( pangolin::PANGO_CTRL + 's',
                                        boost::bind( &Gui::_CTRL_S, this) );

     // toggle help
    pangolin::RegisterKeyPressCallback( 'h', boost::bind( &Gui::_KEYBOARD_H, this ) );
    // toggle keypoints
    pangolin::RegisterKeyPressCallback( 'k', boost::bind( &Gui::_KEYBOARD_K, this ) );
    // toggle previous frames
    pangolin::RegisterKeyPressCallback( 'p', boost::bind( &Gui::_KEYBOARD_P, this ) );
    // toggle correspondences
    pangolin::RegisterKeyPressCallback( 'c', boost::bind( &Gui::_KEYBOARD_C, this ) );
    // toggle feature tracks
    pangolin::RegisterKeyPressCallback( 't', boost::bind( &Gui::_KEYBOARD_T, this ) );
    // toggle landmark info
    pangolin::RegisterKeyPressCallback( 'i', boost::bind( &Gui::_KEYBOARD_I, this ) );
}

/////////////////////////////////////////////////////////////////////////
void Gui::MouseMotion( int x, int y )
{

    // "v" is obviously the viewport
    int w = m_StereoView.v.w/2; // 2 images
    int h = m_StereoView.v.h/2;

    int imw = m_nImageWidth;
    int imh = m_nImageHeight;

    y = y - m_StereoView.v.b;
    x = imw*(float)x/w;
    y = imh*(1.0 - (float)(y-h)/h);

    Measurement selectedCurMeasurement;
    Measurement selectedPrevMeasurement;

    unsigned int selectedId;

    m_glMeasurements.FindClosest(
                      (float)x, (float)y, 10.0f,
                      selectedCurMeasurement,
                      selectedPrevMeasurement,
                      selectedId );

    unsigned char* pPatchCurLeft =
             _LoadMeasurementImagePatch( selectedCurMeasurement, 0, m_CurImageLeft.Data() );
    unsigned char* pPatchCurRight =
             _LoadMeasurementImagePatch( selectedCurMeasurement, 1, m_CurImageRight.Data() );
    unsigned char* pPatchPrevLeft =
             _LoadMeasurementImagePatch( selectedPrevMeasurement, 0, m_PrevImageLeft.Data() );
    unsigned char* pPatchPrevRight =
             _LoadMeasurementImagePatch( selectedPrevMeasurement, 1, m_PrevImageRight.Data() );

    if( selectedId < 4 ) {

        LandmarkId lmkId;

        switch( selectedId ){
            case 0:
            case 1: lmkId = selectedCurMeasurement.m_nId.m_nLandmarkId; break;
            case 2:
            case 3: lmkId = selectedPrevMeasurement.m_nId.m_nLandmarkId; break;
        }

        m_PatchView.SetPatches(
                m_mActiveLandmarks[lmkId].Patch(),
                pPatchCurLeft,
                pPatchCurRight,
                pPatchPrevLeft,
                pPatchPrevRight,
                selectedId
              );
        m_LandmarkInfoView.SetActiveLandmark( lmkId );

       if( pPatchCurLeft   ) { delete pPatchCurLeft; }
       if( pPatchCurRight  ) { delete pPatchCurRight; }
       if( pPatchPrevLeft  ) { delete pPatchPrevLeft; }
       if( pPatchPrevRight ) { delete pPatchPrevRight; }
    }
}

///////////////////////////////////////////////////////////////////////////////
unsigned char* Gui::_LoadMeasurementImagePatch(
                         Measurement&   z,
                         unsigned int   cam,
                         unsigned char* data
                       )
{
    unsigned char* patch = NULL;

     if( !z.IsFlagged( cam, UnInitialized ) &&
         !z.IsFlagged( cam, NoFeaturesToMatch) &&
         !z.IsFlagged( cam, OutsideFOV ) ) {

        patch = new unsigned char[81];
        LoadInterpolatedPatch( data,
                               m_nImageWidth,
                               m_nImageHeight,
                               z.m_dPixel[cam][0],
                               z.m_dPixel[cam][1],
                               patch,
                               m_PatchView.GetPatchWidth(),
                               m_PatchView.GetPatchWidth());
    }

    return patch;
}

///////////////////////////////////////////////////////////////////////////////
void Gui::_SaveDataToFile()
{
    //==========================================================
    // Save Landmarks with observations
    //==========================================================

    std::vector< Landmark* > vpLandmarks;
    Measurement              z;

    // go through the map and get all the landmarks
    m_pRenderMap->GetPtrsToLandmarks( vpLandmarks );

    // [lm_ref_frame, lm_ref_frame_time, lmk_local_id,
    //  msr_frame,     msr_frame_time,  mrm_local_id,
    //   u, v, lmk_in_inverse_depth]
    std::ofstream fout;
    fout.open("landmarks_data.csv");

    for(unsigned int ii=0; ii < vpLandmarks.size(); ++ii ) {

        LandmarkId&      id  = vpLandmarks[ii]->Id();
        Eigen::Vector4d& X   = vpLandmarks[ii]->Pos();
        SlamFramePtr     lmf = m_pRenderMap->GetFramePtr( id.m_nRefFrameId );


        const std::vector<MeasurementId>& track = vpLandmarks[ii]->GetFeatureTrackRef();

        for( unsigned int jj=0; jj < track.size(); ++jj ){


            m_pRenderMap->GetMeasurement( track[jj], z );

            if( !z.HasGoodMeasurement(0) ) continue;

            SlamFramePtr zf = m_pRenderMap->GetFramePtr( z.m_nId.m_nFrameId );

            fout << lmf->Id() << "," << lmf->Time() << "," << id.m_nLandmarkIndex << ",";
            fout << zf->Id()  << "," << zf->Time()  << "," << z.m_nId.m_nLocalIndex << ",";
            fout << z.m_dPixel[0][0] << "," << z.m_dPixel[0][1] << ",";
            fout << X(0) << "," << X(1) << "," << X(2) << "," << X(3) << std::endl;

        }
    }

    fout.close();

    //==========================================================
    // Save estimated poses
    //==========================================================
    std::map< unsigned int, Eigen::Matrix4d > mAbsolutePoses;
    std::map< unsigned int, Eigen::Matrix4d >::iterator pose;

    m_pRenderMap->GetAbsolutePosesToDepth( mAbsolutePoses, 0);

    fout.open("poses.csv");

    for( pose = mAbsolutePoses.begin(); pose != mAbsolutePoses.end(); ++pose ){
        Eigen::Matrix<double,6,1> cart = mvl::T2Cart( pose->second );
        fout << pose->first << "," << cart(0) << "," << cart(1) << "," << cart(2) << ",";
        fout << cart(3) << "," << cart(4) << "," << cart(5) << std::endl;
    }

    fout.close();

}

///////////////////////////////////////////////////////////////////////////////
void Gui::_UpdateCameraView()
{
/*
     Eigen::Vector6d dBBox;
     Eigen::Vector3d dCenter;
     Eigen::Vector2d dz1,dz2,dz3,dz4;
     Eigen::Vector3d dx1,dx2,dx3,dx4;

    m_glPath.GetBBoxInfo(dBBox,dCenter);

    // 3d points from 3d bounding box
    double bh = 0.5*(dBBox(3)-dBBox(2));
    double bw = 0.5*(dBBox(1)-dBBox(0));
    double r  = sqrt(bh*bh + bw*bw);        // rad is half the diag of the bbox
    double cx = dCenter(0);
    double cy = dCenter(1);
    double cz = dCenter(2);

    if( (r == 0.0 || r == m_dBBoxRadius) && !bReset ) return;

    m_dBBoxRadius = r;

    // points in robotics frame
    dx1 << cx,   cy-r, cz; // left
    dx2 << cx-r, cy,   cz; // bottom center
    dx3 << cx,   cy+r, cz; // right
    dx4 << cx,   cy,   cz; // center

    mvl::Aero2VisionInplace(dx1);
    mvl::Aero2VisionInplace(dx2);
    mvl::Aero2VisionInplace(dx3);
    mvl::Aero2VisionInplace(dx4);

    // Set desired location for 2d points
    double w = this->w();
    double h = this->h();
    double cx2d = w/2.0;
    double cy2d = 0.65*h;

    dz1 << cx2d - w*0.45, cy2d;
    dz2 << cx2d, cy2d + cy2d*0.6;
    dz3 << cx2d + w*0.45, cy2d;
    dz4 << cx2d, cy2d;

    // Solve localization problem
    vector<Eigen::Matrix4d> vPoses;
    mvl::SolveThreePointPose(dz1,dz2,dz3,dx1,dx2,dx3,m_dWindowCameraK.inverse(),vPoses);

    // validate pose
    Eigen::Vector2d p;
    Eigen::Matrix4d T;
    Eigen::Matrix4d validT = Eigen::Matrix4d::Identity();
    Eigen::Vector3d dDownDirection;
    double minError = DBL_MAX;
    double downtest;
    double error;
    int kk;

    double	dMData[9] = { 0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0,
                          1.0, 0.0, 0.0 };
    Eigen::Matrix3d M( dMData );


    for(unsigned int ii = 0; ii < vPoses.size(); ii++)
    {

        T = vPoses[ii];

        //Transform to robotics frame
        T.block<3,3>(0,0) = M*vPoses[ii].block<3,3>(0,0)*(M.transpose());
        T.block<3,1>(0,3) = M*vPoses[ii].block<3,1>(0,3);

        //cout << T << endl;
        dDownDirection = T.block<3,1>(0,2);
        downtest = dDownDirection.dot(Eigen::Vector3d(0,0,1));

        // In this function T and 3D Point are both in Vision frame
        p = mvl::Project3dTo2dVisionFrame( m_dWindowCameraK, vPoses[ii], dx4 );

        error = (p-dz4).squaredNorm();

        //cout << "error: " << error <<endl;

        if(error < 1e-6 && error < minError && downtest > 0.0){
        //if(downtest > 0.0){
            validT = T;
            minError = error;
            kk = ii;
        }
    }

    vPoses.clear();

    if(validT == Eigen::Matrix4d::Identity())
        return;

    // test other points

    p = mvl::Project3dTo2dVisionFrame( m_dWindowCameraK, vPoses[kk], dx1 );
    error = (p-dz1).norm();
    if(error > 1e-8) return;
     p = mvl::Project3dTo2dVisionFrame( m_dWindowCameraK, vPoses[kk], dx2 );
    error = (p-dz2).norm();
    if(error > 1e-8) return;
     p = mvl::Project3dTo2dVisionFrame( m_dWindowCameraK, vPoses[kk], dx3 );
    error = (p-dz3).norm();
    if(error > 1e-8) return;

    // set viewing camera
    double eyex    = validT(0,3);
    double eyey    = validT(1,3);
    double eyez    = validT(2,3);
    double targetx = validT(0,3) + validT(0,0);
    double targety = validT(1,3) + validT(1,0);
    double targetz = validT(2,3) + validT(2,0);
    double upx     = -validT(0,2);
    double upy     = -validT(1,2);
    double upz     = -validT(2,2);

    // check that motion is smooth (unless we are reseting the camera)
    if(!bReset)
    {
        double motion = (Eigen::Vector3d(eyex,eyey,eyez) - m_dCamPosition).norm();

        if( motion > 2.0)
            return;

    }

    LookAt(eyex,eyey,eyez,targetx,targety,targetz,upx,upy,upz);
*/
}

#endif	/* GUI_H */

