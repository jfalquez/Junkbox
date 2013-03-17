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
        m_pMap = pMap;
        m_bIsInit = false;

    }

    void DrawCanonicalObject()
    {
        std::cout << "Is init? " << m_bIsInit << std::endl;
        if( m_bIsInit == false ) {
            if( m_pMap->NumFrames() > 0 ) {
                FramePtr pF = m_pMap->GetFramePtr(0);
                unsigned int nImgWidth = pF->GetImageWidth();
                unsigned int nImgHeight = pF->GetImageHeight();
                m_pVBO = new pangolin::GlBufferCudaPtr( pangolin::GlArrayBuffer, nImgWidth*nImgHeight, GL_FLOAT,
                                                        4, cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
                m_pCBO = new pangolin::GlBufferCudaPtr( pangolin::GlArrayBuffer, nImgWidth*nImgHeight, GL_UNSIGNED_BYTE,
                                                        4, cudaGraphicsMapFlagsWriteDiscard, GL_STREAM_DRAW );
                m_pIBO = new pangolin::GlBuffer();
                pangolin::MakeTriangleStripIboForVbo( *m_pIBO, nImgWidth, nImgHeight );


                // Copy point cloud into VBO
                {
                    std::cout << "Starting VBO ... " << std::endl;
//                    Eigen::Matrix3d                 K = CamModel_D.K( ui_nPyrLevel );
                    pangolin::CudaScopedMappedPtr var( *m_pVBO );
                    Gpu::Image< float, Gpu::TargetDevice, Gpu::Manage > Vbo( nImgWidth, nImgHeight );
                    Vbo.MemcpyFromHost( pF->GetDepthImagePtr() );
//                    Gpu::Image<float> Vbo( (float*)pF->GetDepthImagePtr(), nImgWidth, nImgHeight );
                    Gpu::Image< float4 > dVbo( (float4*)*var, nImgWidth, nImgHeight );
                    Gpu::DepthToVbo( dVbo, Vbo, 570, 570, 320, 240 );
                    std::cout << "... done." << std::endl;
                }

                // Generate CBO
                {
                    std::cout << "Starting CBO ... " << std::endl;
                    pangolin::CudaScopedMappedPtr var( *m_pCBO );
//                    Gpu::Image< unsigned char, Gpu::TargetDevice, Gpu::Manage >  Cbo( nImgWidth, nImgHeight );
//                    Cbo.MemcpyFromHost( pF->GetGreyImagePtr() );
                    Gpu::Image< unsigned char >  Cbo( pF->GetGreyImagePtr(), nImgWidth, nImgHeight );
                    Gpu::Image< uchar4 > dCbo( (uchar4*)*var, nImgWidth, nImgHeight );
                    Gpu::ConvertImage<uchar4,unsigned char>( dCbo, Cbo );
                    std::cout << "... done." << std::endl;
                }

                m_bIsInit = true;
            }
        } else {
            glPushAttrib(GL_ENABLE_BIT);
            glDisable(GL_LIGHTING);

            pangolin::RenderVboIboCbo( *m_pVBO, *m_pIBO, *m_pCBO, true, false );

            glPopAttrib();
        }
    }

protected:
    bool                            m_bIsInit;
    DenseMap*                       m_pMap;
    pangolin::GlBufferCudaPtr*      m_pVBO;
    pangolin::GlBufferCudaPtr*      m_pCBO;
    pangolin::GlBuffer*             m_pIBO;
};

#endif
