#ifndef _GLPATH_H_
#define _GLPATH_H_

#include <SceneGraph/GLObject.h>

#define MAT4_COL_MAJOR_DATA(m) (Eigen::Matrix<double,4,4,Eigen::ColMajor>(m).data())

unsigned int g_nPoseDisplay = 5;

/////////////////////////////////////////////////////////////////////////////
// Code to render the vehicle path
class GLPath : public SceneGraph::GLObject
{
public:
    GLPath()
    {
        m_bInitGLComplete = false;
        m_dBaseFrame = Eigen::Matrix4d::Identity();
    }

    ~GLPath()
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

        glPushAttrib(GL_ENABLE_BIT);

        // draw at origin
        glPushMatrix();
        glMultMatrixd( MAT4_COL_MAJOR_DATA( m_dBaseFrame ) );
        glCallList( m_nDrawListId );

        glDisable( GL_LIGHTING );
        glEnable( GL_DEPTH_TEST );

        glEnable(GL_LINE_SMOOTH);

        glLineWidth(1);
        int start = 0;
        if( g_nPoseDisplay != 0 ) {
            if( m_vPoses.size() > g_nPoseDisplay ) {
                start = m_vPoses.size() - g_nPoseDisplay;
            }
        }
        for( int ii = 0; ii < (int)m_vPoses.size(); ii++ ) {
            glPushMatrix();
            glMultMatrixd( MAT4_COL_MAJOR_DATA( m_vPoses[ii] ) );
            if( ii >= start ) {
                glCallList( m_nDrawListId );
            }
            glPopMatrix();
        }
        glPopMatrix();

        glPushMatrix();
        glMultMatrixd( MAT4_COL_MAJOR_DATA( m_dBaseFrame ) );
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(1);
        glColor3f( 1.0, 1.0, 0.0 );


        glBegin(GL_LINE_STRIP);
        for( int ii = 0; ii < (int)m_vPoses.size(); ii++ ) {
            glVertex3d( m_vPoses[ii](0,3), m_vPoses[ii](1,3), m_vPoses[ii](2,3) );
        }
        glEnd();
        glPopMatrix();
        glPopAttrib();
    }

    void InitReset()
    {
        m_vPoses.clear();
    }

    std::vector< Eigen::Matrix4d >& GetPathRef()
    {
        return m_vPoses;
    }

    void PushPose( Eigen::Matrix4d dPose )
    {
        m_vPoses.push_back( dPose );
    }

    void PushPose( Eigen::Vector6d dPose )
    {
        Eigen::Matrix4d T;
        T = mvl::Cart2T(dPose);
        m_vPoses.push_back( T );
    }

    void SetRotation( const Eigen::Vector3d& dR )
    {
        // convert from degrees to radians
        Eigen::Vector3d dRR = dR * 0.0174532925;

        m_dBaseFrame.block<3,3>(0,0) = mvl::Cart2R( dRR );
    }


private:
    bool _GLInit()
    {
        m_nDrawListId = glGenLists(1);

        // compile drawlist for a pose
        glNewList( m_nDrawListId, GL_COMPILE );

        glDepthMask( GL_TRUE );

        glEnable(GL_DEPTH_TEST);

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
    GLuint                          m_nDrawListId;
    bool                            m_bInitGLComplete;
    Eigen::Matrix4d                 m_dBaseFrame;
    std::vector< Eigen::Matrix4d >  m_vPoses;
};

#endif

