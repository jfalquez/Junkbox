#ifndef _GLPATH_H_
#define _GLPATH_H_

#include <SceneGraph/GLObject.h>

#include <DenseMap/DenseMap.h>


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
        m_fPointSize = 5.0;
        m_nPoseDisplay = 0;
        m_bDrawAxis = true;
        m_bDrawLines = true;
        m_bDrawPoints = true;
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

        glDisable( GL_LIGHTING );
        glEnable( GL_DEPTH_TEST );

        glEnable(GL_LINE_SMOOTH);

        glLineWidth(1);

        // draw at base pose
        glPushMatrix();
        glMultMatrixd( MAT4_COL_MAJOR_DATA( m_pMap->GetPathBasePose() ) );
        glCallList( m_nDrawListId );


        std::map<unsigned int, Eigen::Matrix4d> vPoses;

        m_pMap->GenerateAbsolutePoses( vPoses );

        if( !vPoses.empty() ) {
            // look for first pose of map, which is our "true" origin
            Eigen::Matrix4d dOrigin = _TInv( vPoses[0] );

            if( m_bDrawAxis ) {
                std::map<unsigned int, Eigen::Matrix4d>::iterator it;
                for( it = vPoses.begin(); it != vPoses.end(); it++ ) {
                    Eigen::Matrix4d&    Pose = it->second;

                    glPushMatrix();
                    glMultMatrixd( MAT4_COL_MAJOR_DATA( dOrigin * Pose ) );
                    glCallList( m_nDrawListId );
                    glPopMatrix();
                }
            }

            if( m_bDrawLines ) {
                glEnable( GL_LINE_SMOOTH );
                glLineWidth( 1 );
                glColor4f( m_fLineColor(0), m_fLineColor(1), m_fLineColor(2), m_fLineColor(3) );

                glBegin( GL_LINE_STRIP );
                std::map<unsigned int, Eigen::Matrix4d>::iterator it;
                for( it = vPoses.begin(); it != vPoses.end(); it++ ) {
                    Eigen::Matrix4d    Pose = dOrigin * it->second;
                    glVertex3f( Pose(0,3), Pose(1,3), Pose(2,3) );
                }
                glEnd();
            }
        }
        /* */


        /*
        std::vector< Eigen::Matrix4d >& vPath = m_pMap->GetPathRef();

        if( m_bDrawAxis ) {
            int start = 0;
            if( m_nPoseDisplay != 0 ) {
                if( vPath.size() > m_nPoseDisplay ) {
                    start = vPath.size() - m_nPoseDisplay;
                }
            }
            glPushMatrix();
            for( int ii = 0; ii < (int)vPath.size(); ii++ ) {
                glMultMatrixd( MAT4_COL_MAJOR_DATA( vPath[ii] ) );
                if( ii >= start ) {
                    glCallList( m_nDrawListId );
                }
            }
            glPopMatrix();
        }

        if( m_bDrawPoints ) {
            glPointSize( m_fPointSize );
            glEnable( GL_POINT_SMOOTH );
            glEnable( GL_BLEND );
            glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
            glColor4f( m_fPointColor(0), m_fPointColor(1), m_fPointColor(2), m_fPointColor(3) );
            Eigen::Matrix4d T;
            T.setIdentity();
            glBegin( GL_POINTS );
            for( int ii = 0; ii < (int)vPath.size(); ii++ ) {
                T = T * vPath[ii];
                glVertex3f( T(0,3), T(1,3), T(2,3) );
            }
            glEnd();
        }

        if( m_bDrawLines ) {
            glEnable( GL_LINE_SMOOTH );
            glLineWidth( 1 );
            glColor4f( m_fLineColor(0), m_fLineColor(1), m_fLineColor(2), m_fLineColor(3) );

            Eigen::Matrix4d T;
            T.setIdentity();
            glBegin( GL_LINE_STRIP );
            for( int ii = 0; ii < (int)vPath.size(); ii++ ) {
                T = T * vPath[ii];
                glVertex3f( T(0,3), T(1,3), T(2,3) );
            }
            glEnd();
        }
        /* */

        glPopMatrix();
        glPopAttrib();
    }

    void InitReset( DenseMap *pMap )
    {
        m_pMap = pMap;
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


private:

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool _GLInit()
    {
        m_nDrawListId = glGenLists(1);

        // compile drawlist for a pose
        glNewList( m_nDrawListId, GL_COMPILE );

        glDepthMask( GL_TRUE );

        glEnable( GL_DEPTH_TEST );

        glEnable( GL_BLEND );
        glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );

        if( m_bDrawAxis ) {
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
        }

        glEndList();
        m_bInitGLComplete = true;
        return true;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline Eigen::Matrix4d _TInv( Eigen::Matrix4d T )
    {
        // calc Hji = [ Hij(1:3,1:3).' -Hij(1:3,1:3).'*Hij(1:3,4); 0 0 0 1 ];
        Eigen::Matrix4d invT;
        invT.block<3,3>(0,0) =  T.block<3,3>(0,0).transpose();
        invT.block<3,1>(0,3) = -T.block<3,3>(0,0).transpose() * T.block<3,1>(0,3);
        invT.row(3) = Eigen::Vector4d( 0, 0, 0, 1 );
        return invT;
    }


private:
    DenseMap*                       m_pMap;
    bool                            m_bDrawLines;
    bool                            m_bDrawAxis;
    bool                            m_bDrawPoints;
    GLuint                          m_nDrawListId;
    float                           m_fPointSize;
    unsigned int                    m_nPoseDisplay;
    bool                            m_bInitGLComplete;
    Eigen::Vector4f                 m_fLineColor;
    Eigen::Vector4f                 m_fPointColor;
};

#endif
