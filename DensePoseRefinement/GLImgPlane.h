
#ifndef GLIMGPLANE_H
#define GLIMGPLANE_H

#include <SceneGraph/GLObject.h>
//#include <SceneGraph/GLHelpers.h>

class GLImgPlane:
public SceneGraph::GLObject
{
    public:

        // ////////////////////////////////////////////////////////////////////
        GLImgPlane() {
            m_nTex                = 0;
            m_pImageData          = 0;
            m_bTextureInitialized = false;
            m_dBaseFrame          = Eigen::Matrix4d::Identity();
        }

        // ////////////////////////////////////////////////////////////////////
        void Init() {
        }

        // ////////////////////////////////////////////////////////////////////
        void SetBaseFrame(const Eigen::Vector6d & dPose) {
            m_dBaseFrame = mvl::Cart2T(dPose);
        }

        // ////////////////////////////////////////////////////////////////////
        void SetImage(unsigned char * pImageData,
                      int             w,
                      int             h,
                      int             nFormat,
                      int             nType) {
            if (!pImageData) {
                return;
            }

            int          nBpp     = SceneGraph::GLBytesPerPixel(nFormat, nType);
            unsigned int nMemSize = nBpp * w * h;

            m_pImageData          = (unsigned char *) malloc(nMemSize);
            m_bTextureInitialized = false;

            memcpy(m_pImageData, pImageData, nMemSize);

            m_nImageWidth  = w;
            m_nImageHeight = h;
            m_nFormat      = nFormat;
            m_nType        = nType;
        }

        // ////////////////////////////////////////////////////////////////////
        void DrawCanonicalObject() {
            float nTop    = -204.8;
            float nBottom = 0;
            float nLeft   = 0;
            float nRight  = 136.2;

            _InitTexture();
            glPushAttrib(GL_ENABLE_BIT);
            glPushMatrix();
            glMultMatrixd(m_dBaseFrame.data());
            glDisable(GL_LIGHTING);
            glDepthMask(GL_TRUE);
            glEnable(GL_DEPTH_TEST);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glShadeModel(GL_SMOOTH);
            glEnable(GL_TEXTURE_RECTANGLE_ARB);
            glBindTexture(GL_TEXTURE_RECTANGLE_ARB, m_nTex);
            glColor4f(1.0, 1.0, 1.0, 1.0);
            glBegin(GL_QUADS);
            glTexCoord2f(0.0, 0.0);
            glVertex3f(0.0, nRight, nTop);
            glTexCoord2f(m_nImageWidth, 0.0);
            glVertex3f(0.0, nRight, nBottom);
            glTexCoord2f(m_nImageWidth, m_nImageHeight);
            glVertex3f(0.0, nLeft, nBottom);
            glTexCoord2f(0.0, m_nImageHeight);
            glVertex3f(0.0, nLeft, nTop);
            glEnd();
            glBindTexture(GL_TEXTURE_RECTANGLE_ARB, 0);
            glDisable(GL_TEXTURE_RECTANGLE_ARB);
            glPopMatrix();
            glPopAttrib();
        }

    private:

        // ////////////////////////////////////////////////////////////////////
        void _InitTexture() {

            if (m_bTextureInitialized) {
                return;
            }

            if (!m_pImageData) {
                return;
            }

            m_bTextureInitialized = true;
            m_nTex                = SceneGraph::GenerateAndBindRectTextureID(m_nImageWidth, m_nImageHeight, m_nFormat, m_nType, m_pImageData);
        }

    private:
        Eigen::Matrix4d m_dBaseFrame;
        bool            m_bTextureInitialized;
        unsigned char * m_pImageData;
        int             m_nImageWidth;
        int             m_nImageHeight;
        int             m_nType;      // gl
        int             m_nFormat;    // gl
        unsigned int    m_nTex;       // < Texture ID associated with OpenGL texture.
};
#endif // GLIMGPLANE_H