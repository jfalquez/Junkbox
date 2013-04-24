#ifndef _GLMAP_H_
#define _GLMAP_H_

#include <SceneGraph/GLObject.h>

#include <pangolin/pangolin.h>
#include <pangolin/glvbo.h>

#include <DenseMap/DenseMap.h>

#define MAT4_COL_MAJOR_DATA(m) (Eigen::Matrix<double,4,4,Eigen::ColMajor>(m).data())


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class GLMap : public SceneGraph::GLObject
{
public:

    GLMap()
    { }

    void InitReset( DenseMap* pMap )
    {
        m_bInitIBO          = false;
        m_pMap              = pMap;
        m_dLastModifiedTime = 0;
    }

    void DrawCanonicalObject()
    {
        // check if mas has changed since last we updated VBOs
        if( m_dLastModifiedTime != m_pMap->GetLastModifiedTime() ) {

            // get image dimensions
            // TODO add an option to see hi res versus low res VBOs
            // for hi res, not all can be displayed so do something
            FramePtr pFrame = m_pMap->GetFramePtr(0);
//            const unsigned int nImgWidth    = pFrame->GetImageWidth();
            const unsigned int nImgWidth    = pFrame->GetThumbWidth();
//            const unsigned int nImgHeight   = pFrame->GetImageHeight();
            const unsigned int nImgHeight   = pFrame->GetThumbHeight();

            // allocate single IBO which will be shared
            if( m_bInitIBO == false ) {
                m_pIBO = new pangolin::GlBuffer();
                pangolin::MakeTriangleStripIboForVbo( *m_pIBO, nImgWidth, nImgHeight );

                m_bInitIBO = true;
            }

            // allocate and populate VBO and CBO
            for( unsigned int ii = m_pVBO.size(); ii < m_pMap->GetNumFrames(); ++ii ) {
                pFrame = m_pMap->GetFramePtr( ii );

                // check if this frame is keyframe
                if( pFrame->IsKeyframe() ) {

                    const unsigned int nId = pFrame->GetId();

                    // check if we have already created a VBO for this keyframe
                    if( m_pVBO.find( nId ) == m_pVBO.end() ) {

                        pangolin::GlBuffer* pVBO = new pangolin::GlBuffer( pangolin::GlArrayBuffer, nImgWidth * nImgHeight, GL_FLOAT, 4, GL_STREAM_DRAW );
                        pangolin::GlBuffer* pCBO = new pangolin::GlBuffer( pangolin::GlArrayBuffer, nImgWidth * nImgHeight, GL_UNSIGNED_BYTE, 4, GL_STREAM_DRAW );

                        float VBO[nImgWidth * nImgHeight * 4];
                        Eigen::Matrix3d DepthK = m_pMap->GetDepthCameraK( 4 );
                        _DepthToVBO( (float*)pFrame->GetDepthThumbPtr(), nImgWidth, nImgHeight, DepthK(0,0), DepthK(1,1), DepthK(0,2), DepthK(1,2), VBO );
                        pVBO->Upload( VBO, nImgWidth * nImgHeight * 4 * 4);

                        unsigned char CBO[nImgWidth * nImgHeight * 4];
                        _GreyToCBO( (unsigned char*)pFrame->GetGreyThumbPtr(), nImgWidth, nImgHeight, CBO );
                        pCBO->Upload( CBO, nImgWidth * nImgHeight * 4);

                        m_pVBO[ nId ] = pVBO;
                        m_pCBO[ nId ] = pCBO;

                    }
                }
            }
            m_dLastModifiedTime = m_pMap->GetLastModifiedTime();
        }

        std::map<unsigned int, Eigen::Matrix4d>& vPoses = m_pMap->GetInternalPath();


        if( !vPoses.empty() ) {
            Eigen::Matrix4d& dPathOrientation =  m_pMap->GetPathOrientation();
            glPushMatrix();
            glMultMatrixd( MAT4_COL_MAJOR_DATA( dPathOrientation ) );

            // look for first pose of map, which is our "true" origin
            Eigen::Matrix4d dOrigin = _TInv( vPoses[0] );

            // draw!
            glPushAttrib( GL_ENABLE_BIT );
            glDisable( GL_LIGHTING );

            for( auto it = vPoses.begin(); it != vPoses.end(); ++it ) {
                unsigned int        nId = it->first;
                Eigen::Matrix4d&    Pose = it->second;

                if( m_pVBO.find( nId ) != m_pVBO.end() ) {
                    glPushMatrix();
                    glMultMatrixd( MAT4_COL_MAJOR_DATA( dOrigin * Pose ) );
                    pangolin::RenderVboIboCbo( *m_pVBO[nId], *m_pIBO, *m_pCBO[nId], true, true );
                    glPopMatrix();
                }
            }
            glPopAttrib();
            glPopMatrix();
        }

    }

    void ToggleShow()
    {
        if( IsVisible() ) {
            SetVisible( false );
        } else {
            SetVisible( true );
        }
    }


private:

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void _DepthToVBO(
            float*      pDepth,
            int         nWidth,
            int         nHeight,
            float       fx,
            float       fy,
            float       cx,
            float       cy,
            float*      pVBO
        )
    {
        /*
        const float fMaxDistance = 5.0;
        for( int ii = 0; ii < nHeight; ++ii ) {
            for( int jj = 0; jj < nWidth; ++jj ) {
                const unsigned int nIdx = (ii * nWidth) + jj;
                const unsigned int nVboIdx = nIdx * 4;
                float fDistanceWeight = ( ((cx - abs(jj - cx)) / cx ) + ((cy - abs(ii - cy)) / cy) ) / 2;
                if( pDepth[nIdx] < fMaxDistance * fDistanceWeight ) {
                    pVBO[nVboIdx]   = 0.0/0.0;
                    pVBO[nVboIdx+1] = 0.0/0.0;
                    pVBO[nVboIdx+2] = 0.0/0.0;
                } else {
                    pVBO[nVboIdx]   = pDepth[nIdx];
                    pVBO[nVboIdx+1] = pDepth[nIdx] * (jj-cx) / fx;
                    pVBO[nVboIdx+2] = pDepth[nIdx] * (ii-cy) / fy;
                }
                pVBO[nVboIdx+3] = 1;
            }
        }
        /* */

        for( int ii = 0; ii < 10; ++ii ) {
            for( int jj = 0; jj < nWidth; ++jj ) {
                const unsigned int nIdx = (ii * nWidth) + jj;
                const unsigned int nVboIdx = nIdx * 4;
                pVBO[nVboIdx]   = 0.0/0.0;
                pVBO[nVboIdx+1] = 0.0/0.0;
                pVBO[nVboIdx+2] = 0.0/0.0;
                pVBO[nVboIdx+3] = 1;
            }
        }

        /* */
        for( int ii = 10; ii < nHeight; ++ii ) {
            float lastDepth = -1;
            for( int jj = 0; jj < nWidth; ++jj ) {
                const unsigned int nIdx = (ii * nWidth) + jj;
                const unsigned int nVboIdx = nIdx * 4;
                if( lastDepth == -1 ) {
                    lastDepth = pDepth[nIdx];
                }
                if( fabs(lastDepth - pDepth[nIdx] ) > 0.3 ) {
                    pVBO[nVboIdx]   = 0.0/0.0;
                    pVBO[nVboIdx+1] = 0.0/0.0;
                    pVBO[nVboIdx+2] = 0.0/0.0;
                } else {
                    pVBO[nVboIdx]   = pDepth[nIdx];
                    pVBO[nVboIdx+1] = pDepth[nIdx] * (jj-cx) / fx;
                    pVBO[nVboIdx+2] = pDepth[nIdx] * (ii-cy) / fy;
                }
                pVBO[nVboIdx+3] = 1;
                lastDepth = pDepth[nIdx];
            }
        }
        /* */
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    inline void _GreyToCBO(
            unsigned char*  pGrey,
            int             nWidth,
            int             nHeight,
            unsigned char*  pCBO
        )
    {
        for( int ii = 0; ii < nHeight; ++ii ) {
            for( int jj = 0; jj < nWidth; ++jj ) {
                const unsigned int nIdx = (ii * nWidth) + jj;
                const unsigned int nVboIdx = nIdx * 4;
                pCBO[nVboIdx]   = pGrey[nIdx];
                pCBO[nVboIdx+1] = pGrey[nIdx];
                pCBO[nVboIdx+2] = pGrey[nIdx];
                pCBO[nVboIdx+3] = 255;
            }
        }
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


protected:
    DenseMap*                                           m_pMap;
    double                                              m_dLastModifiedTime;
    bool                                                m_bInitIBO;
    pangolin::GlBuffer*                                 m_pIBO;
    std::map< unsigned int, pangolin::GlBuffer* >       m_pVBO;
    std::map< unsigned int, pangolin::GlBuffer* >       m_pCBO;
};

#endif
