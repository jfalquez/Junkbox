#ifndef _GLMAP_H_
#define _GLMAP_H_

#include <SceneGraph/GLObject.h>

#include <pangolin/pangolin.h>
#include <pangolin/glcuda.h>
#include <pangolin/glsl.h>
#include <pangolin/glvbo.h>

#include <kangaroo/kangaroo.h>

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
        m_pMap      = pMap;
        m_bIsInit   = false;

    }

    void DrawCanonicalObject()
    {
        if( m_pMap->NumFrames() < 0 ) {
//        if( m_pMap->GetCurrentKeyframe() ) {
            FramePtr pKeyframe = m_pMap->GetCurrentKeyframe();
            unsigned int nImgWidth = pKeyframe->GetImageWidth();
            unsigned int nImgHeight = pKeyframe->GetImageHeight();

            // allocate memory
            if( m_bIsInit == false ) {
                m_pVBO = new pangolin::GlBuffer( pangolin::GlArrayBuffer, nImgWidth*nImgHeight, GL_FLOAT, 4, GL_STREAM_DRAW );
                m_pCBO = new pangolin::GlBuffer( pangolin::GlArrayBuffer, nImgWidth*nImgHeight, GL_UNSIGNED_BYTE, 4, GL_STREAM_DRAW );
                m_pIBO = new pangolin::GlBuffer();
                pangolin::MakeTriangleStripIboForVbo( *m_pIBO, nImgWidth, nImgHeight );
                m_bIsInit = true;
            }

            // TODO convert depth to VBO
            m_pVBO->Upload( pKeyframe->GetDepthImagePtr(), nImgWidth*nImgHeight * 4 * 4);
            m_pCBO->Upload( pKeyframe->GetGreyImagePtr(), nImgWidth*nImgHeight * 4);

            // draw!
            glPushAttrib( GL_ENABLE_BIT );
            glDisable( GL_LIGHTING );

            pangolin::RenderVboIboCbo( *m_pVBO, *m_pIBO, *m_pCBO, true, false );

            glPopAttrib();
        }
    }

protected:
    bool                            m_bIsInit;
    DenseMap*                       m_pMap;
    pangolin::GlBuffer*             m_pVBO;
    pangolin::GlBuffer*             m_pCBO;
    pangolin::GlBuffer*             m_pIBO;
};

#endif
