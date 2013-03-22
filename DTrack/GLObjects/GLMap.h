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
        m_bInitIBO  = false;
        m_pMap      = pMap;
    }

    void DrawCanonicalObject()
    {
        if( m_pVBO.size() < m_pMap->GetNumFrames() ) {

            // get image dimensions
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
            for( int ii = m_pVBO.size(); ii < m_pMap->GetNumFrames(); ++ii ) {
                pFrame = m_pMap->GetFramePtr( ii );

                pangolin::GlBuffer* pVBO = new pangolin::GlBuffer( pangolin::GlArrayBuffer, nImgWidth * nImgHeight, GL_FLOAT, 4, GL_STREAM_DRAW );
                pangolin::GlBuffer* pCBO = new pangolin::GlBuffer( pangolin::GlArrayBuffer, nImgWidth * nImgHeight, GL_UNSIGNED_BYTE, 4, GL_STREAM_DRAW );

                float VBO[nImgWidth * nImgHeight * 4];
                // TODO obtain these intrinsics from SOMEWHERE
//                _DepthToVBO( (float*)pFrame->GetDepthImagePtr(), nImgWidth, nImgHeight, 570, 570, 320, 240, VBO );
                _DepthToVBO( (float*)pFrame->GetDepthThumbPtr(), nImgWidth, nImgHeight, 570/16, 570/16, 320/16, 240/16, VBO );
                pVBO->Upload( VBO, nImgWidth * nImgHeight * 4 * 4);

                unsigned char CBO[nImgWidth * nImgHeight * 4];
//                _GreyToCBO( (unsigned char*)pFrame->GetGreyImagePtr(), nImgWidth, nImgHeight, CBO );
                _GreyToCBO( (unsigned char*)pFrame->GetGreyThumbPtr(), nImgWidth, nImgHeight, CBO );
                pCBO->Upload( CBO, nImgWidth * nImgHeight * 4);

                m_pVBO.push_back( pVBO );
                m_pCBO.push_back( pCBO );

            }
        }

        std::map<unsigned int, Eigen::Matrix4d> vPoses;

        /* */
        m_pMap->GenerateAbsolutePoses( vPoses );

        // draw!
        glPushAttrib( GL_ENABLE_BIT );
        glDisable( GL_LIGHTING );

        std::map<unsigned int, Eigen::Matrix4d>::iterator it;
        for( it = vPoses.begin(); it != vPoses.end(); it++ ) {
            unsigned int        nId = it->first;
            Eigen::Matrix4d&    Pose = it->second;

            glPushMatrix();
            glMultMatrixd( MAT4_COL_MAJOR_DATA( Pose ) );
            pangolin::RenderVboIboCbo( *m_pVBO[nId], *m_pIBO, *m_pCBO[nId], true, true );
            glPopMatrix();
        }
        /* */

        /*
        m_pMap->GenerateRelativePoses( vPoses );

        // draw!
        glPushAttrib( GL_ENABLE_BIT );
        glDisable( GL_LIGHTING );

        glPushMatrix();
        for( int ii = 0; ii < vPoses.size(); ii++ ) {
            Eigen::Matrix4d&    Pose = vPoses[ii];

            glMultMatrixd( MAT4_COL_MAJOR_DATA( Pose ) );
            pangolin::RenderVboIboCbo( *m_pVBO[ii], *m_pIBO, *m_pCBO[ii], true, true );
        }
        glPopMatrix();
        /* */

        glPopAttrib();
    }


private:
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
    }

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



protected:
    bool                                    m_bInitIBO;
    DenseMap*                               m_pMap;
    pangolin::GlBuffer*                     m_pIBO;
    std::deque < pangolin::GlBuffer* >      m_pVBO;
    std::deque < pangolin::GlBuffer* >      m_pCBO;
    std::deque < Eigen::Matrix4d >          m_vPoses;
};

#endif
