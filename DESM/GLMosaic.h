#ifndef GLMOSAIC_H
#define GLMOSAIC_H

#include <SimpleGui/GLObject.h>

class GLMosaic : public GLObject
{
public:

	GLMosaic( )
	{
		m_nWidth = 100;
		m_nHeight = 100;
		m_dBaseFrame = Eigen::Matrix4d::Identity( );
	}

	void Init( unsigned int nWidth, unsigned int nHeight )
	{
		m_nWidth = nWidth % 2 == 0 ? nWidth : nWidth + 1;
		m_nHeight = nHeight % 2 == 0 ? nHeight : nHeight + 1;
		srand( time( NULL ) );
		for( int ii = 0; ii < 10000; ii++ ) {
			r[ii] = (rand( ) % 100) / 100.0;
			g[ii] = (rand( ) % 100) / 100.0;
			b[ii] = (rand( ) % 100) / 100.0;
		}
	}

	void draw( )
	{
		glPushAttrib( GL_ENABLE_BIT );
		glPushMatrix( );
		glMultMatrixd( m_dBaseFrame.data( ) );

		glDisable( GL_LIGHTING );
		glDepthMask( GL_TRUE );
		glEnable( GL_DEPTH_TEST );
		glEnable( GL_BLEND );
		glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
		glShadeModel( GL_SMOOTH );

		/*
		// Debugging Point
		glPointSize(1.0);
		glBegin( GL_POINTS );
		glColor3f( 1.0, 0.0, 0.0 );
		glVertex3f( 0, 0, 0 );
		glEnd();
		/* */

		/* */
		glBegin( GL_QUADS );

		m_nColorId = 0;
		unsigned int Delta = 20;
		for( float y = 0; y <= m_nHeight; y += Delta ) {
			for( float x = 0; x <= m_nWidth; x += Delta ) {
				glColor3f( r[m_nColorId], g[m_nColorId], b[m_nColorId] );
				m_nColorId++;
				if( m_nColorId >= 10000 ) {
					m_nColorId = 0;
				}
				glVertex3f( 0, x, y );

				glColor3f( r[m_nColorId], g[m_nColorId], b[m_nColorId] );
				m_nColorId++;
				if( m_nColorId >= 10000 ) {
					m_nColorId = 0;
				}
				glVertex3f( 0, x, y + Delta );

				glColor3f( r[m_nColorId], g[m_nColorId], b[m_nColorId] );
				m_nColorId++;
				if( m_nColorId >= 10000 ) {
					m_nColorId = 0;
				}
				glVertex3f( 0, x + Delta, y + Delta );

				glColor3f( r[m_nColorId], g[m_nColorId], b[m_nColorId] );
				m_nColorId++;
				if( m_nColorId >= 10000 ) {
					m_nColorId = 0;
				}
				glVertex3f( 0, x + Delta, y );
			}
		}
		glEnd( );
		/* */
		glPopMatrix( );
		glPopAttrib( );
	}

	void SetBaseFrame( const Eigen::Matrix4d& dPose )
	{
		m_dBaseFrame = dPose;
	}


private:
	Eigen::Matrix4d m_dBaseFrame;
	unsigned int m_nWidth;
	unsigned int m_nHeight;
	unsigned int m_nColorId;
	float r[10000];
	float g[10000];
	float b[10000];
};

#endif // GLMOSAIC_H
