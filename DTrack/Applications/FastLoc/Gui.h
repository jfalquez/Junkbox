#ifndef __GUI_H_
#define	__GUI_H_

#include <mutex>

#include <opencv.hpp>

#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>

#include <DenseMap/DenseMap.h>
#include <DenseFrontEnd/DenseFrontEnd.h>

#include <GUI/GLPath.h>
#include <GUI/GLMap.h>
#include <GUI/AnalyticsView.h>
#include <GUI/TimerView.h>

#include "GuiConfig.h"


GuiConfig           guiConfig;
extern              DenseFrontEndConfig feConfig;

enum GuiState { PLAYING, STEPPING, PAUSED, RESETTING, RESET_COMPLETE, QUIT };

class Gui
{

public:

    Gui();
    Gui( const std::string& sWindowName, const int nWidth, const int nHeight );

    void Init();
    void InitReset();
    void Run();
    void CopyMapChanges( DenseMap& rMap );

    // data updates
    void UpdateImages( const cv::Mat& LiveGrey );

    void UpdateTimer(
                    int                                      nTimeWindowSize,
                    std::vector< std::string>                vNames,
                    std::vector< std::pair<double,double> >  vTimes
                    );

    void UpdateAnalytics( std::map< std::string,  std::pair< double, double > >& mData );

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
        State = (State == PAUSED) ? PLAYING : PAUSED;
    }

    void _CTRL_R()
    {
        State = RESETTING;
        while( State != RESET_COMPLETE ) {
            usleep(10000);
        }
        InitReset(); // not called from elsewhere
        State = PAUSED;
    }


/////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
public:
    volatile GuiState               State;


private:
    bool                            m_bMapDirty;

    DenseFrontEnd*                  m_pFrontEnd;
    DenseMap*                       m_pRenderMap;          // re-allocated on reset. for now.
    DenseMap*                       m_pChangesBufferMap;   // re-allocated on reset. for now.

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
    GLMap                           m_glMap;

    // objects for view container
    SceneGraph::ImageView           m_LiveGrey;
    SceneGraph::ImageView           m_KeyGrey;
    SceneGraph::ImageView           m_KeyDepth;
    SceneGraph::ImageView           m_LoopClosure;

    // extras
    AnalyticsView                   m_AnalyticsView;
    TimerView                       m_TimerView;

    std::mutex                      m_Mutex;
};




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementation
Gui::Gui()
{
     Gui( "DTrack Application", 640, 480 );
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Gui::Gui( const std::string& sWindowName, const int nWidth, const int nHeight )
{
    m_sWindowName        = sWindowName;
    m_nWindowWidth       = nWidth;
    m_nWindowHeight      = nHeight;
    m_nImageWidth        = 0;
    m_nImageHeight       = 0;
    m_bMapDirty          = false;
    m_pChangesBufferMap  = NULL;
    m_pRenderMap         = NULL;
    State                = PLAYING;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Gui::Init()
{
    // create OpenGL window in single line thanks to GLUT
    pangolin::CreateGlutWindowAndBind( m_sWindowName, m_nWindowWidth, m_nWindowHeight );

    // init GLEW
    glewInit();

    // register keyboard callbacks
    _RegisterKeyboardCallbacks();

    // side panel
    const unsigned int nPanelSize = 350;
    pangolin::CreatePanel("ui").SetBounds( 0, 1, 0, pangolin::Attach::Pix(nPanelSize) ).Show(false);
    pangolin::Var<unsigned int>     ui_nBlur( "ui.Blur", 1, 0, 5 );
    pangolin::Var<bool>             ui_bBreakEarly( "ui.Break Early", false, true );
    pangolin::Var<float>            ui_fBreakErrorThreshold( "ui.Break Early Error Threshold", 0.8, 0, 2 );
    pangolin::Var<float>            ui_fNormC( "ui.Norm C", 10, 0, 100);
    pangolin::Var<bool>             ui_bDiscardMaxMin( "ui.Discard Max-Min Pix Values", true, true );

    // configure 3d view
    SceneGraph::GLSceneGraph::ApplyPreferredGlSettings();
    glClearColor( 0, 0, 0, 0 );
    m_gl3dGraph.AddChild( &m_glGrid );
    m_gl3dGraph.AddChild( &m_glMap );
    m_gl3dGraph.AddChild( &m_glPath );

    m_gl3dRenderState.SetProjectionMatrix( pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 3000) );
    m_gl3dRenderState.SetModelViewMatrix( pangolin::ModelViewLookAt(-20, 0, -30, 0, 0, 0, pangolin::AxisNegZ) );

    m_View3d.SetBounds( 0.25, 1.0, pangolin::Attach::Pix(280), 1.0, 640.0f/480.0f );
    m_View3d.SetAspect( 640.0 / 480.0 );
    m_View3d.SetHandler( new SceneGraph::HandlerSceneGraph( m_gl3dGraph, m_gl3dRenderState, pangolin::AxisNone ) );
    m_View3d.SetDrawFunction( SceneGraph::ActivateDrawFunctor( m_gl3dGraph, m_gl3dRenderState ) );

    // configure view container -- for images
    m_ViewContainer.SetBounds( 0, 0.25, pangolin::Attach::Pix(nPanelSize), 1 );
    m_ViewContainer.SetLayout( pangolin::LayoutEqual );
    m_LiveGrey.SetAspect(1.0);
    m_ViewContainer.AddDisplay( m_LiveGrey );
    m_ViewContainer.AddDisplay( m_KeyGrey );
    m_ViewContainer.AddDisplay( m_KeyDepth );

    // configure extras
    m_TimerView.SetBounds( 0.4, 1.0, 0, pangolin::Attach::Pix(nPanelSize) );
    m_AnalyticsView.SetBounds( 0, 0.4, 0, pangolin::Attach::Pix(nPanelSize) );

    // add views to base window
    pangolin::DisplayBase().AddDisplay( m_View3d );
    pangolin::DisplayBase().AddDisplay( m_ViewContainer );
    pangolin::DisplayBase().AddDisplay( m_AnalyticsView );
    pangolin::DisplayBase().AddDisplay( m_TimerView );

    InitReset();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Gui::InitReset()
{
    m_Mutex.lock();
    if( m_pRenderMap )        { delete m_pRenderMap; }
    if( m_pChangesBufferMap ) { delete m_pChangesBufferMap; }

    m_pRenderMap        = new DenseMap; // this is used by GLObjects
    m_pChangesBufferMap = new DenseMap; // this receives data from the front end
    m_bMapDirty         = false;
    m_Mutex.unlock();

    // set properties
    m_glGrid.SetNumLines( guiConfig.g_nNumGridLines );
    m_glGrid.SetLineSpacing( 3.0 );

    // init-reset objects
    m_glMap.InitReset( m_pRenderMap );
    m_glPath.InitReset( m_pRenderMap );

    // init-reset extras
    m_TimerView.InitReset();
    m_AnalyticsView.InitReset();
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Gui::Run()
{
    while( !pangolin::ShouldQuit() && State != QUIT ) {

         if( m_bMapDirty ) {
            // the map has changed update it before rendering again
            m_Mutex.lock();
            m_pRenderMap->CopyMapChanges( *m_pChangesBufferMap );
            m_bMapDirty = false;
            m_Mutex.unlock();
         }

         // update gui variables
         m_glGrid.SetNumLines( guiConfig.g_nNumGridLines );
         m_glPath.SetPoseDisplay( guiConfig.g_nNumPosesToShow );
         m_glPath.SetSphereRadius( feConfig.g_fCloseKeyframeNorm );

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        pangolin::FinishGlutFrame();

        usleep(1000000 / 120);
    }

    State = QUIT;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Gui::CopyMapChanges(
        DenseMap&       rMap        //< Input: Map we are copying from
    )
{
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_bMapDirty = m_pChangesBufferMap->CopyMapChanges( rMap );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Gui::UpdateImages(
        const cv::Mat&      LiveGrey
    )
{
    if( m_nImageWidth == 0 ) {
        m_nImageWidth  = LiveGrey.cols;
        m_nImageHeight = LiveGrey.rows;
    }

    m_LiveGrey.SetImage( LiveGrey.data, m_nImageWidth, m_nImageHeight, GL_RGB8, GL_LUMINANCE );

    FramePtr pKeyframe = m_pRenderMap->GetCurrentKeyframe();
    if( pKeyframe ) {
        cv::Mat KeyGrey, KeyDepth;
        pKeyframe->CopyGreyImageTo( KeyGrey );
        pKeyframe->CopyDepthImageTo( KeyDepth );
        m_KeyGrey.SetImage( KeyGrey.data, m_nImageWidth, m_nImageHeight, GL_RGB8, GL_LUMINANCE, GL_UNSIGNED_BYTE );
        // TODO currently a rough normalization.. make it nicer
        KeyDepth = KeyDepth / guiConfig.g_fMaxDepth;
        m_KeyDepth.SetImage( KeyDepth.data, m_nImageWidth, m_nImageHeight, GL_RGB8, GL_LUMINANCE, GL_FLOAT );
    }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Gui::UpdateTimer(
        int                                     nTimeWindowSize,
        std::vector< std::string >              vsNames,
        std::vector< std::pair<double,double> > vTimes
        )
{
    m_TimerView.Update( nTimeWindowSize, vsNames, vTimes );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Gui::UpdateAnalytics(
        std::map< std::string, std::pair< double, double > >&    mData
    )
{
    m_AnalyticsView.Update( mData );
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Gui::_RegisterKeyboardCallbacks()
{
    // step once
    pangolin::RegisterKeyPressCallback( pangolin::PANGO_SPECIAL + GLUT_KEY_RIGHT,
                                        std::bind( &Gui::_RIGHT_ARROW, this) );

    // play / pause
    pangolin::RegisterKeyPressCallback( ' ',
                                        std::bind( &Gui::_SPACE_BAR, this) );

    // reset app
    pangolin::RegisterKeyPressCallback( pangolin::PANGO_CTRL + 'r',
                                        std::bind( &Gui::_CTRL_R, this) );

    // print map
    pangolin::RegisterKeyPressCallback( 'm',
                                        [this](){ m_pRenderMap->ImportMap(); }
                                        );

    // export map
    pangolin::RegisterKeyPressCallback( pangolin::PANGO_CTRL + 'm',
                                        [this](){ m_pRenderMap->ExportMap(); }
                                        );

    // toggle showing side panel
    pangolin::RegisterKeyPressCallback('~', [this](){ static bool showpanel = false; showpanel = !showpanel;
        if(showpanel) { m_TimerView.Show(false); m_AnalyticsView.Show(false);  } else
            { m_TimerView.Show(true); m_AnalyticsView.Show(true); }
                    pangolin::Display("ui").Show(showpanel); } );


     // toggle showing map
    pangolin::RegisterKeyPressCallback( '1', [this](){ m_glMap.ToggleShow(); } );
}


#endif	/* GUI_H */
