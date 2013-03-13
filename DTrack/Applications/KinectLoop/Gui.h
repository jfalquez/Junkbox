#ifndef __GUI_H_
#define	__GUI_H_

#include <boost/bind.hpp>
#include <opencv.hpp>

#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>

#include <GLObjects/GLPath.h>

#include "GuiConfig.h"


GuiConfig           guiConfig;

enum GuiState { PLAYING, STEPPING, PAUSED, RESETTING, RESET_COMPLETE, GETSTATE, QUIT };

class Gui
{

    public:

        Gui();
        Gui( const std::string& sWindowName, const int nWidth, const int nHeight );

        void Init();
        void InitReset();
        void Run();
//        void CopyMapChanges( Map& rMap );

        // data updates
        void UpdateImages( const cv::Mat& LiveGrey );

        void SetState( GuiState s )
        {
            State = s;
        }

    private:

        void _RegisterKeyboardCallbacks();

        void _RIGHT_ARROW()
        {
            State = STEPPING;
        }

        void _SPACE_BAR()
        {
            State = (State == PAUSED)?PLAYING:PAUSED;
        }

        void _CTRL_R()
        {
            State = RESETTING;
            while( State != RESET_COMPLETE ){
                usleep(10000);
            }
            InitReset(); // not called from elsewhere
            State = PAUSED;
        }

        void _KEYBOARD_H()
        {
            //m_HelpView.ToggleShow();
        }

    public:
        volatile GuiState               State;

    private:

        bool                            m_bMapDirty;

//        DenseMap*                         m_pChangesBufferMap;   // re-allocated on reset. for now.
//        DenseMap*                         m_pRenderMap;          // re-allocated on reset. for now.

        int                             m_nWindowWidth;
        int                             m_nWindowHeight;
        int                             m_nImageWidth;
        int                             m_nImageHeight;
        std::string                     m_sWindowName;

        pangolin::View                  m_View3d;
        pangolin::View                  m_ViewContainer;

        pangolin::OpenGlRenderState     m_gl3dRenderState;
        SceneGraph::GLSceneGraph        m_gl3dGraph;

        // objects for 3d view
        SceneGraph::GLGrid              m_glGrid;
        GLPath                          m_glPath;

        // objects for view container
        SceneGraph::ImageView           m_LiveGrey;
        SceneGraph::ImageView           m_KeyGrey;
        SceneGraph::ImageView           m_KeyDepth;
        SceneGraph::ImageView           m_Debug;

        boost::mutex                    m_Mutex;
};

/////////////////////////////////////////////////////////////////////////
// Implementation
/////////////////////////////////////////////////////////////////////////
Gui::Gui()
{
     Gui( "RPG Application", 640, 480 );
}

/////////////////////////////////////////////////////////////////////////
Gui::Gui( const std::string& sWindowName, const int nWidth, const int nHeight )
{
    m_sWindowName        = sWindowName;
    m_nWindowWidth       = nWidth;
    m_nWindowHeight      = nHeight;
    m_nImageWidth        = 0;
    m_nImageHeight       = 0;
    m_bMapDirty          = false;
//    m_pChangesBufferMap  = NULL;
//    m_pRenderMap         = NULL;
    State                = PLAYING;
}

/////////////////////////////////////////////////////////////////////////
void Gui::Init()
{
    // create OpenGL window in single line thanks to GLUT
    pangolin::CreateGlutWindowAndBind( m_sWindowName, m_nWindowWidth, m_nWindowHeight );

    // register keyboard callbacks
    _RegisterKeyboardCallbacks();

    // side panel
    pangolin::CreatePanel("ui").SetBounds( 0, 1, 0, pangolin::Attach::Pix(280) );


    // configure 3d view
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    m_gl3dGraph.AddChild( &m_glGrid );
//    m_gl3dGraph.AddChild( &m_glPath );

    m_gl3dRenderState.SetProjectionMatrix( pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000) );
    m_gl3dRenderState.SetModelViewMatrix( pangolin::ModelViewLookAt(-6, 0, -10, 1, 0, 0, pangolin::AxisNegZ) );

    m_View3d.SetBounds( 0.4, 1.0, pangolin::Attach::Pix(280), 1.0, 640.0f/480.0f );
    m_View3d.SetAspect( 640.0 / 480.0 );
    m_View3d.SetHandler( new SceneGraph::HandlerSceneGraph( m_gl3dGraph, m_gl3dRenderState, pangolin::AxisNone) );
    m_View3d.SetDrawFunction( SceneGraph::ActivateDrawFunctor( m_gl3dGraph, m_gl3dRenderState ) );



    // configure view container
    m_ViewContainer.SetBounds( 0, 0.4, pangolin::Attach::Pix(280), 1 );
    m_ViewContainer.SetLayout( pangolin::LayoutEqual );
    m_ViewContainer.AddDisplay( m_LiveGrey );
    m_ViewContainer.AddDisplay( m_KeyGrey );
    m_ViewContainer.AddDisplay( m_KeyDepth );
    m_ViewContainer.AddDisplay( m_Debug );

    /*
    const unsigned int nNumViews = 1;
    for( unsigned int ii = 0; ii < nNumViews; ++ii ) {
        pangolin::View& subView = pangolin::CreateDisplay();
        pangoView.SetAspect((double)nImgWidth/nImgHeight);
        m_ViewContainer.AddDisplay( subView );
    }
    */



    // add views to base window
    pangolin::DisplayBase().AddDisplay( m_View3d );
    pangolin::DisplayBase().AddDisplay( m_ViewContainer );

    InitReset();
}

///////////////////////////////////////////////////////////////////////////////
void Gui::InitReset()
{
    m_Mutex.lock();
//    if( m_pChangesBufferMap ){ delete m_pChangesBufferMap; }
//    if( m_pRenderMap )       { delete m_pRenderMap; }

//    m_pChangesBufferMap = new Map; // this receives data from the front end
//    m_pRenderMap        = new Map; // this is used by GLObjects
    m_bMapDirty         = false;
    m_Mutex.unlock();

    // initreset objects
    m_glGrid.SetNumLines( guiConfig.g_nNumGridLines);
    m_glGrid.SetLineSpacing( 10.0 );
//    m_glPath.InitReset( m_pRenderMap );


    // hide or show things
//    m_PrevImageLeft.Show( false );
}

///////////////////////////////////////////////////////////////////////////////
void Gui::Run()
{
    while( !pangolin::ShouldQuit() ) {

         if( m_bMapDirty ) {
            // the map has changed update it before rendering again
            m_Mutex.lock();
//            m_pRenderMap->CopyMapChanges( *m_pChangesBufferMap );
            m_bMapDirty = false;
            m_Mutex.unlock();
         }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pangolin::FinishGlutFrame();

        usleep(1000000 / 120);
    }

    State = QUIT;
}

///////////////////////////////////////////////////////////////////////////////
/*
void Gui::CopyMapChanges( DenseMap& rMap )
{
    boost::mutex::scoped_lock lock(m_Mutex);
    m_pChangesBufferMap->CopyMapChanges( rMap );
    m_bMapDirty = true;
}
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Gui::UpdateImages( const cv::Mat& LiveGrey )
{
    if( m_nImageWidth == 0 ) {
        m_nImageWidth  = LiveGrey.cols;
        m_nImageHeight = LiveGrey.rows;
    }

    m_LiveGrey.SetImage( LiveGrey.data, m_nImageWidth, m_nImageHeight, GL_INTENSITY, GL_LUMINANCE );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

     // toggle help
    pangolin::RegisterKeyPressCallback( 'h', boost::bind( &Gui::_KEYBOARD_H, this ) );
}


#endif	/* GUI_H */
