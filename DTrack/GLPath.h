#ifndef _GLPATH_H_
#define _GLPATH_H_

#include <SceneGraph/GLObject.h>

#define MAT4_COL_MAJOR_DATA(m) (Eigen::Matrix<double,4,4,Eigen::ColMajor>(m).data())


/////////////////////////////////////////////////////////////////////////////
// Code to render the vehicle path
class GLPath : public SceneGraph::GLObject
{
public:
    GLPath()
    {
        m_bInitGLComplete = false;
        m_fLineColor(0) = 1.0;
        m_fLineColor(1) = 1.0;
        m_fLineColor(2) = 0.0;
        m_fLineColor(3) = 1.0;
        m_fPointColor(0) = 1.0;
        m_fPointColor(1) = 0.0;
        m_fPointColor(2) = 0.0;
        m_fPointColor(3) = 1.0;
        m_fPointSize = 0.0;
        m_nPoseDisplay = 1;
        m_bDrawAxis = true;
        m_bDrawLines = true;
        m_bDrawPoints = true;
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

        if( m_bDrawAxis ) {
            int start = 0;
            if( m_nPoseDisplay != 0 ) {
                if( m_vPoses.size() > m_nPoseDisplay ) {
                    start = m_vPoses.size() - m_nPoseDisplay;
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
        }

        if( m_bDrawPoints ) {
            glPointSize( m_fPointSize );
            glEnable( GL_POINT_SMOOTH );
            glEnable( GL_BLEND );
            glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
            glColor4f( m_fPointColor(0), m_fPointColor(1), m_fPointColor(2), m_fPointColor(3) );
            glBegin( GL_POINTS );
            for( int ii = 0; ii < (int)m_vPoses.size(); ii++ ) {
                glVertex3f( m_vPoses[ii](0,3), m_vPoses[ii](1,3), m_vPoses[ii](2,3) );
            }
            glEnd();
        }

        glPopMatrix();

        if( m_bDrawLines ) {
            glPushMatrix();
            glMultMatrixd( MAT4_COL_MAJOR_DATA( m_dBaseFrame ) );
            glEnable(GL_LINE_SMOOTH);
            glLineWidth(1);
//            glColor4f( m_fLineColor(0), m_fLineColor(1), m_fLineColor(2), m_fLineColor(3) );

            glBegin(GL_LINE_STRIP);
            for( int ii = 0; ii < (int)m_vPoses.size(); ii++ ) {
                glColor4f( m_fLineColor(0), m_fLineColor(1), m_fLineColor(2), m_fLineColor(3) );
                glVertex3d( m_vPoses[ii](0,3), m_vPoses[ii](1,3), m_vPoses[ii](2,3) );
            }
            glEnd();
            glPopMatrix();
        }

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

    void SetLineColor( float R, float G, float B, float A = 1.0 )
    {
        m_fLineColor(0) = R;
        m_fLineColor(1) = G;
        m_fLineColor(2) = B;
        m_fLineColor(3) = A;
    }

    void SetPointColor( float R, float G, float B, float A = 1.0 )
    {
        m_fPointColor(0) = R;
        m_fPointColor(1) = G;
        m_fPointColor(2) = B;
        m_fPointColor(3) = A;
    }

    void SetPoseDisplay( unsigned int Num )
    {
        m_nPoseDisplay = Num;
    }

    void SetPointSize( float Size )
    {
        m_fPointSize = Size;
    }

    void DrawLines( bool Val )
    {
        m_bDrawLines = Val;
    }

    void DrawPoints( bool Val )
    {
        m_bDrawPoints = Val;
    }

    void DrawAxis( bool Val )
    {
        m_bDrawAxis = Val;
    }

    std::vector< float >& GetPointBlendRef() {
        return m_vPointBlend;
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
    bool                            m_bDrawLines;
    bool                            m_bDrawAxis;
    bool                            m_bDrawPoints;
    GLuint                          m_nDrawListId;
    float                           m_fPointSize;
    unsigned int                    m_nPoseDisplay;
    bool                            m_bInitGLComplete;
    Eigen::Matrix4d                 m_dBaseFrame;
    Eigen::Vector4f                 m_fLineColor;
    Eigen::Vector4f                 m_fPointColor;
    std::vector< Eigen::Matrix4d >  m_vPoses;
    std::vector< float >            m_vPointBlend;
};

#endif
