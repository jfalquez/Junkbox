#ifndef GLIMGPLANE_H
#define GLIMGPLANE_H

#include <SimpleGui/GLObject.h>

class GLImgPlane : public GLObject
{
public:

	GLImgPlane( )
	{
		m_nTex = 0;
		m_pImageData = 0;
		m_bTextureInitialized = false;
	}

	void Init( unsigned int nWidth, unsigned int nHeight )
	{
		m_nWidth = nWidth;
		m_nHeight = nHeight;
	}

	void draw( )
	{
		//		glEnable( GL_DEPTH_TEST );
		//		glEnable( GL_LIGHTING );
		//		glEnable( GL_LIGHT0 );
		//		glEnable( GL_COLOR_MATERIAL );
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
		glShadeModel( GL_SMOOTH );
		glLoadIdentity( );

		glBegin( GL_TRIANGLES );
		float y = -m_nHeight / 2;
		while( y < m_nHeight / 2 ) {
			float x = (-m_nWidth / 2) + ((rand( ) % 100) + 1) / 10;
			;
			while( x < m_nWidth / 2 ) {
				float r, g, b;
				r = (rand( ) % 100) / 100;
				g = (rand( ) % 100) / 100;
				b = (rand( ) % 100) / 100;
				glColor3f( r, g, b );
				glVertex3f( x, y, 0.0 );
				x += ((rand( ) % 100) + 1) / 10;
			}
			// generate a number between 1.0 - 10.0
			y += ((rand( ) % 100) + 1) / 10;

		}
		glEnd( );
	}



private:
        bool           	m_bTextureInitialized;
        unsigned char*  m_pImageData;
        int             m_nImageWidth;
        int             m_nImageHeight;
        unsigned int   	m_nTex;        	//< Texture ID associated with OpenGL texture.
        unsigned int	m_nBorder;		//< Border size.

};

#endif // GLIMGPLANE_H
