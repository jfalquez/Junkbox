/*
 * File:   AnalyticsView.h
 * Author: alonso
 *
 * Created on January 18, 2013, 11:46 AM
 */

#ifndef _ANALYTICS_VIEW_H_
#define	_ANALYTICS_VIEW_H_

#include <cstdlib>
#include <vector>
#include <map>
#include <string>
#include <mutex>

#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>

#include "GLVarHistory.h"
#include "ColorPalette.h"

/////////////////////////////////////////////////////////////////////////////
class AnalyticsView : public pangolin::View
{
    public:

        ///////////////////////////////////////////////////////////////////////////////
        AnalyticsView()
        {

        }

        ///////////////////////////////////////////////////////////////////////////////
        ~AnalyticsView()
        {
            Clear();
        }

        ///////////////////////////////////////////////////////////////////////////////
        void InitReset( unsigned int uVarHistoryLength = 60 )
        {
            Clear();

            m_uVarHistoryLength = uVarHistoryLength;
        }

        ///////////////////////////////////////////////////////////////////////////////
        void Clear()
        {

            std::map< std::string, GLVarHistory* >::iterator itVar;

            for( itVar = m_mData.begin(); itVar != m_mData.end(); ++itVar ) {
                // draw: (width,height, offset_x, offset_y)
                delete itVar->second;
            }

            m_mData.clear();
        }

        ///////////////////////////////////////////////////////////////////////////////
        virtual void Resize( const pangolin::Viewport& parent )
        {
            pangolin::View::Resize(parent);

            m_Ortho = pangolin::ProjectionMatrixOrthographic(0, v.w, v.h, 0.5, 0, 1E4 );

            // recompute these for adjusting rendering
            m_fWidth      = (float)v.w;
            m_fHeight     = (float)v.h;
        }

        ///////////////////////////////////////////////////////////////////////////////
        void Render()
        {
            std::lock_guard<std::mutex> lock(m_Mutex);

            _SetRenderingState();

             // draw widget bounding box
            glColor4f(0.0,0.0,0.0,0.6);
            glBegin(GL_QUADS);
                glVertex2f( 0.0,  0.0 );
                glVertex2f( m_fWidth, 0.0 );
                glVertex2f( m_fWidth, m_fHeight );
                glVertex2f( 0.0,  m_fHeight );
            glEnd();

            glLineWidth(1);
            glColor4f(1.0,1.0,1.0,1.0);
            glBegin(GL_LINE_LOOP);
                glVertex2f( 0.0,  0.0 );
                glVertex2f( m_fWidth, 0.0 );
                glVertex2f( m_fWidth, m_fHeight );
                glVertex2f( 0.0,  m_fHeight );
            glEnd();

            float fVarHeight = m_fHeight/m_mData.size();
            float fVarWidth  = m_fWidth;

            Palette& colors = m_ColorPalette.GetPaletteRef( eNewPalette );

            std::map< std::string, GLVarHistory* >::iterator itVar;

            int ii;
            for( ii = 0, itVar = m_mData.begin(); itVar != m_mData.end(); ++itVar, ii++ ) {
                // draw: (width,height, offset_x, offset_y)
                itVar->second->Draw( fVarWidth, fVarHeight, 0.0, ii * fVarHeight, colors[ii] );
            }

            glPopAttrib();
        }


        ///////////////////////////////////////////////////////////////////////////////
        void Update( std::map< std::string, std::pair< double, double > >& mData )
        {
            std::lock_guard<std::mutex> lock(m_Mutex);

            for( auto itInput = mData.begin(); itInput != mData.end(); ++itInput ) {
                const std::string&  sVar = itInput->first;
                std::pair<double, double>& dVal = itInput->second;

                auto it = m_mData.find( sVar );

                if( it == m_mData.end() ) {
                    // create variable
                    m_mData[ sVar ] = new GLVarHistory;
                    m_mData[ sVar ]->InitReset( sVar, m_uVarHistoryLength );
                }

                m_mData[ sVar ]->Update( dVal.first, dVal.second );
            }
        }

    private:

        ///////////////////////////////////////////////////////////////////////////////
        void _SetRenderingState()
        {
            glPushAttrib(GL_ENABLE_BIT);
            glDisable(GL_LIGHTING);
           // glEnable(GL_BLEND);
           // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
           // glEnable(GL_LINE_SMOOTH);
            //glDisable(GL_DEPTH_TEST);

            // Activate viewport
            this->Activate();

            // Load orthographic projection matrix to match image
            glMatrixMode(GL_PROJECTION);
            m_Ortho.Load();

            // Reset ModelView matrix
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
        }


    private:

        unsigned int  m_uVarHistoryLength;
        float         m_fWidth;
        float         m_fHeight;
        ColorPalette  m_ColorPalette;

        // Data to display
        std::map< std::string, GLVarHistory* > m_mData;

        // Projection matrix
        pangolin::OpenGlMatrix m_Ortho;

        // mutex for blocking drawing while updating data
        std::mutex m_Mutex;
};

#endif	/* ANALYTICSVIEW_H */

