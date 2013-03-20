#ifndef _GLKEYFRAME_H_
#define _GLKEYFRAME_H_

#include <SceneGraph/GLObject.h>

#include <pangolin/pangolin.h>
#include <pangolin/glvbo.h>

#include <DenseMap/DenseMap.h>

#define MAT4_COL_MAJOR_DATA(m) (Eigen::Matrix<double,4,4,Eigen::ColMajor>(m).data())


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class GLKeyframe : public SceneGraph::GLObject
{
public:

    GLKeyframe()
    { }

    void InitReset( DenseMap* pMap )
    {
        m_pMap      = pMap;
        m_bIsInit   = false;

    }

    void DrawCanonicalObject()
    {
        if( m_pMap->GetCurrentKeyframe() ) {
            FramePtr pKeyframe = m_pMap->GetCurrentKeyframe();
            const unsigned int nImgWidth = pKeyframe->GetImageWidth();
            const unsigned int nImgHeight = pKeyframe->GetImageHeight();

            // allocate memory
            if( m_bIsInit == false ) {
                m_pVBO = new pangolin::GlBuffer( pangolin::GlArrayBuffer, nImgWidth * nImgHeight, GL_FLOAT, 4, GL_STREAM_DRAW );
                m_pCBO = new pangolin::GlBuffer( pangolin::GlArrayBuffer, nImgWidth * nImgHeight, GL_UNSIGNED_BYTE, 4, GL_STREAM_DRAW );
                m_pIBO = new pangolin::GlBuffer();
                pangolin::MakeTriangleStripIboForVbo( *m_pIBO, nImgWidth, nImgHeight );
                m_bIsInit = true;
            }

            // only update VBO if keyframe has changed
            if( m_pLastKeyframe != pKeyframe ) {
                float VBO[nImgWidth * nImgHeight * 4];
                // TODO obtain these intrinsics from SOMEWHERE
                _DepthToVBO( (float*)pKeyframe->GetDepthImagePtr(), nImgWidth, nImgHeight, 570, 570, 320, 240, VBO );
                m_pVBO->Upload( VBO, nImgWidth * nImgHeight * 4 * 4);

                unsigned char CBO[nImgWidth * nImgHeight * 4];
                _GreyToCBO( (unsigned char*)pKeyframe->GetGreyImagePtr(), nImgWidth, nImgHeight, CBO );
                m_pCBO->Upload( CBO, nImgWidth * nImgHeight * 4);
            }

            // draw!
            glPushAttrib( GL_ENABLE_BIT );
            glDisable( GL_LIGHTING );

            // TODO obtain global pose from somewhere

            pangolin::RenderVboIboCbo( *m_pVBO, *m_pIBO, *m_pCBO, true, true );

            glPopAttrib();
        }
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
        for( int ii = 0; ii < nHeight; ++ii ) {
            for( int jj = 0; jj < nWidth; ++jj ) {
                const unsigned int nIdx = (ii * nWidth) + jj;
                const unsigned int nVboIdx = nIdx * 4;
                pVBO[nVboIdx]   = pDepth[nIdx];
                pVBO[nVboIdx+1] = pDepth[nIdx] * (jj-cx) / fx;
                pVBO[nVboIdx+2] = pDepth[nIdx] * (ii-cy) / fy;
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
    bool                            m_bIsInit;
    DenseMap*                       m_pMap;
    FramePtr                        m_pLastKeyframe;
    pangolin::GlBuffer*             m_pVBO;
    pangolin::GlBuffer*             m_pCBO;
    pangolin::GlBuffer*             m_pIBO;
};

#endif
