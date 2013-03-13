#ifndef _GLPYRPATH_H_
#define _GLPYRPATH_H_

#include <tuple>

#include <SceneGraph/GLObject.h>

#define MAT4_COL_MAJOR_DATA(m) (Eigen::Matrix<double,4,4,Eigen::ColMajor>(m).data())


/////////////////////////////////////////////////////////////////////////////
// Code to render the vehicle path
class GLPyrPath : public SceneGraph::GLObject
{
public:
    GLPyrPath( unsigned int PyrLevels )
    {
        m_bInitGLComplete = false;
        m_nPyrLevels = PyrLevels;

        Eigen::Vector3f Color;
        Color <<  0.5, 0.0, 1.0;
        m_vColors.push_back( Color );
        Color <<  0.0, 0.0, 1.0;
        m_vColors.push_back( Color );
        Color <<  0.0, 1.0, 0.0;
        m_vColors.push_back( Color );
        Color <<  1.0, 1.0, 0.0;
        m_vColors.push_back( Color );
        Color <<  1.0, 0.5, 0.0;
        m_vColors.push_back( Color );
        Color <<  1.0, 0.0, 0.0;
        m_vColors.push_back( Color );
    }

    ~GLPyrPath()
    {

    }

    /// destroy called from main FLTK thread. GTS TODO finish this approach...
    void Destroy()
    {
        glDeleteLists( m_nDrawListId, 1 ); // bad idea? needs to happen in the GLThread...
    }

    // just draw the path
    void DrawCanonicalObject()
    {
        if ( m_bInitGLComplete == false ){
            if( _GLInit() ){
                return;
            }
        }

        glPushAttrib( GL_ENABLE_BIT );

        glDisable( GL_LIGHTING );
        glEnable( GL_DEPTH_TEST );
        glEnable( GL_LINE_SMOOTH );

        glLineWidth( 2.0 );
        unsigned int nPyrLvl;
        if( m_vPoses.empty() == false ) {
            glBegin( GL_LINE_STRIP );
            for( int ii = 0; ii < m_vPoses.size(); ii++ ) {
                nPyrLvl = std::get<0>( m_vPoses[ii] );
                Eigen::Vector3f& Color = m_vColors[nPyrLvl];
                glColor3f( Color(0), Color(1), Color(2) );
                glPushMatrix();
                Eigen::Matrix4d& Pose = std::get<1>( m_vPoses[ii] );
                glVertex3d( Pose(0,3), Pose(1,3), Pose(2,3) );
                glPopMatrix();
            }
            glEnd();
        }
        glPopAttrib();
    }

    void InitReset()
    {
        m_vPoses.clear();
    }

    void PushPose(
            unsigned int                nPyrLvl,
            const Eigen::Matrix4d&      dPose
            )
    {
        m_vPoses.push_back( std::make_tuple( nPyrLvl, dPose ) );
    }


private:
    bool _GLInit()
    {
        m_nDrawListId = glGenLists(1);

        // compile drawlist for a pose
        glNewList( m_nDrawListId, GL_COMPILE );

        glDepthMask( GL_TRUE );

        glEnable( GL_DEPTH_TEST );

        glEnable( GL_BLEND );
        glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

        glBegin( GL_LINES );

        glColor4f( 1.0, 0, 0, 0.5 );
        glVertex3f( 0.0, 0.0, 0.0 );
        glVertex3f( 1.0, 0.0, 0.0 );

        glColor4f( 0, 0, 1.0, 0.5 );
        glVertex3f( 0.0, 0.0, 0.0 );
        glVertex3f( 0.0, 0.0, 1.0 );

        glColor4f( 0, 1.0, 0, 0.5 );
        glVertex3f( 0.0, 0.0, 0.0 );
        glVertex3f( 0.0, 1.0, 0.0 );

        glEnd();
        glEndList();
        m_bInitGLComplete = true;
        return true;
    }

private:
    GLuint                                                                      m_nDrawListId;
    bool                                                                        m_bInitGLComplete;
    unsigned int                                                                m_nPyrLevels;
    std::vector< std::tuple < unsigned int, Eigen::Matrix4d > >                 m_vPoses;
    std::vector< Eigen::Vector3f >                                              m_vColors;
};

#endif
