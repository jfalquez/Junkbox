#ifndef _TimerView_H_
#define _TimerView_H_

#include <cstdlib>
#include <boost/thread/mutex.hpp>
#include <vector>
#include <deque>
#include <string>

#include <pangolin/pangolin.h>
#include <SceneGraph/SceneGraph.h>

#include "ColorPalette.h"

/////////////////////////////////////////////////////////////////////////////
class TimerView : public pangolin::View
{
    public:

        ///////////////////////////////////////////////////////////////////////////////
        TimerView()
        {
            m_fPercentGraphics     = 0.5;
            m_fPercentFunctionName = 0.8;
            m_bTimerInit           = false;
            m_nDisplayLevels       = 2;
            m_fScale               = 0.0;
            m_fFps                 = 0.0;
            m_sFps                 = "";
        }

        ///////////////////////////////////////////////////////////////////////////////
        ~TimerView(){}

        ///////////////////////////////////////////////////////////////////////////////
        void InitReset()
        {
            m_bTimerInit  = false;
            m_nOffsetY    = 15;
            m_vsNames.clear();
            m_vsTimes.clear();
            m_vHistory.clear();
        }

        ///////////////////////////////////////////////////////////////////////////////
        void Update( int nTimeWindowSize,
                     std::vector< std::string >&  vsNames,
                     std::vector< std::pair<double,double> >& vTimes
                    )
        {
            boost::mutex::scoped_lock lock(m_Mutex);

            if( vTimes.empty() ){
                return;
            }

            m_fStepY  = m_fHeight/vsNames.size();
            m_fStepX = m_fDivision/nTimeWindowSize;

            // fill our member variables with updated values
            m_nTimeWindowSize = nTimeWindowSize;
            m_vsNames              = vsNames;

            std::vector<double> vdAccTimes;
            char                         cBuff[40];

            m_vsTimes.resize(vsNames.size());

            // init accTimes and string vector
            for(int ii=0; ii <(int)vTimes.size(); ++ii) {
                vdAccTimes.push_back(vTimes[ii].second);
                sprintf(cBuff, "%.2f", vTimes[ii].first);
                m_vsTimes[ii].assign(cBuff);
            }

            //compute accumulated processing time
            for(int ii=(int)vdAccTimes.size()-2; ii >= 0; --ii){
                vdAccTimes[ii] += vdAccTimes[ii+1];
            }

            // save new times in the history
            m_vHistory.push_back(vdAccTimes);

            // check if history queue is full
            if( (int)m_vHistory.size() > nTimeWindowSize ){
                m_vHistory.pop_front();
            }

            m_fScale = 0.0;

            if( m_vHistory.size() > 0 ){

                m_fFps = floor((0.8)*m_fFps + (0.2)*(1000.0 / m_vHistory.back().front()));

                sprintf(cBuff, "FPS: %.1f", m_fFps);
                m_sFps.assign(cBuff);

                // compute max value for scaling
                for(int ii = 0; ii < (int)m_vHistory.size(); ii++){
                    if(m_vHistory[ii][0] > m_fScale){
                        m_fScale = m_vHistory[ii][0];
                    }
                }
            }
        }

        ///////////////////////////////////////////////////////////////////////////////
        // overloaded from View
        virtual void Resize( const pangolin::Viewport& parent )
        {

            pangolin::View::Resize(parent);

            m_Ortho = pangolin::ProjectionMatrixOrthographic(0, v.w, v.h, 0.5, 0, 1E4 );

            // recompute these for adjusting rendering
            m_fWidth      = (float)v.w;
            m_fHeight     = (float)v.h;
            m_fDivision   = m_fWidth * m_fPercentGraphics;
            m_fTimeOffset = m_fDivision + ( m_fWidth - m_fDivision) * m_fPercentFunctionName;
            m_fStepY      = ((int)m_vsNames.size() > 0)?m_fHeight/m_vsNames.size():0.0;
            m_fStepX      = m_fDivision/m_nTimeWindowSize;
            m_fScale      = 0.0;
        }

        ///////////////////////////////////////////////////////////////////////////////
        // overloaded from View
        virtual void Render()
        {
            boost::mutex::scoped_lock lock(m_Mutex);

            glPushAttrib(GL_ENABLE_BIT);
            glDisable(GL_LIGHTING);

            // Activate viewport
            this->Activate();

            // Load orthographic projection matrix to match image
            glMatrixMode(GL_PROJECTION);
            m_Ortho.Load();

            // Reset ModelView matrix
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

             // draw widget bounding box BG
            glColor4f( 0.0, 0.0, 0.0, 0.6);
            glBegin(GL_QUADS);
                glVertex2f( 0.0,  0.0 );
                glVertex2f( m_fWidth, 0.0 );
                glVertex2f( m_fWidth, m_fHeight );
                glVertex2f( 0.0,  m_fHeight );
            glEnd();

            glLineWidth(1);

            // draw timed functions names and process time
            //gl_font( 1, 12 );
            Palette& colors = m_ColorPalette.GetPaletteRef( eNewPalette );

            for(unsigned int ii=0; ii<m_vsNames.size(); ++ii) {
                glColor4f( colors[ii][0], colors[ii][1], colors[ii][2], 1.0 );
                glRasterPos2f( m_fDivision+5.0, m_nOffsetY + (m_fStepY*ii) );
                m_glText.Draw( m_vsNames[ii] );
                glColor4f( 1.0, 1.0, 1.0, 1.0 );
                glRasterPos2f( m_fTimeOffset, m_nOffsetY + (m_fStepY*ii) );
                m_glText.Draw( m_vsTimes[ii] );
            }

            // draw process time history
            float fVX1, fVX2, fVX3, fVX4;
            float fVY1, fVY2, fVY3 = 0, fVY4 = 0;

            float fScale = 0.0;

            if(m_fScale > 0.0)
                fScale = m_fHeight/m_fScale;

            unsigned int nInit = m_nTimeWindowSize - m_vHistory.size();

            for( unsigned int ii=0; ii<m_vsNames.size(); ++ii) {
                glColor4f( colors[ii][0], colors[ii][1], colors[ii][2], 0.8 );
                for( unsigned int jj=nInit; jj<m_nTimeWindowSize-1; ++jj) {
                    fVX1 = jj*m_fStepX;
                    fVX2 = jj*m_fStepX;
                    fVX3 = (jj+1)*m_fStepX;
                    fVX4 = (jj+1)*m_fStepX;
                    fVY1 = m_fHeight;
                    fVY2 = m_fHeight-m_vHistory[jj-nInit][ii]*fScale;
                    fVY3 = m_fHeight-m_vHistory[jj-nInit+1][ii]*fScale;
                    fVY4 = m_fHeight;

                    if(ii < m_vsNames.size()-1) {
                        fVY1 -= m_vHistory[jj-nInit][ii+1]*fScale;
                        fVY4 -= m_vHistory[jj-nInit+1][ii+1]*fScale;
                    }
                    glBegin(GL_QUADS);
                    glVertex2f(fVX1, fVY1);
                    glVertex2f(fVX2, fVY2);
                    glVertex2f(fVX3, fVY3);
                    glVertex2f(fVX4, fVY4);
                    glEnd();

                }

                fVX1 = m_fStepX*(m_nTimeWindowSize-1);
                fVX2 = fVX1;
                fVX3 = m_fStepX*m_nTimeWindowSize;
                fVY1 = fVY4;
                fVY2 = fVY3;
                fVY3 = m_nOffsetY + ii*m_fStepY - m_fStepY*0.3;

                glBegin(GL_TRIANGLES);
                    glVertex2f(fVX1,fVY1);
                    glVertex2f(fVX2,fVY2);
                    glVertex2f(fVX3,fVY3);
                glEnd();
            }

            // draw widget bounding box
            glLineWidth(1);
            glColor4f(1.0,1.0,1.0,1.0);
            glBegin(GL_LINE_LOOP);
                glVertex2f( 0.0,  0.0 );
                glVertex2f( m_fWidth, 0.0 );
                glVertex2f( m_fWidth, m_fHeight );
                glVertex2f( 0.0,  m_fHeight );
            glEnd();

            // draw division between function names and graphics
            glBegin(GL_LINES);
                glVertex2f( m_fDivision, 0.0);
                glVertex2f( m_fDivision, m_fHeight);
            glEnd();

            // draw frames per second
            glRasterPos2f(5.0, m_nOffsetY);
            m_glText.Draw(m_sFps);

            // Call base View implementation
            pangolin::View::Render();

            glPopAttrib();
        }

    private:

        // Projection matrix
        pangolin::OpenGlMatrix m_Ortho;

        bool             m_bNewData;
        bool             m_bTimerInit;
        int              m_nDisplayLevels;
        int              m_nFuncNameWidth;
        int              m_nOffsetY;
        unsigned int     m_nTimeWindowSize;
        float            m_fScale;
        float            m_fFps;
        std::string      m_sFps;

        // Render variables
        float  m_fPercentGraphics;
        float  m_fPercentFunctionName;
        float  m_fDivision;
        float  m_fStepY;
        float  m_fStepX;
        float  m_fTimeOffset;
        float  m_fWidth;
        float  m_fHeight;

        SceneGraph::GLText  m_glText;
        ColorPalette        m_ColorPalette;

        std::vector< std::string > m_vsNames;
        std::vector< std::string > m_vsTimes;
        std::deque< std::vector<double>  > m_vHistory;

        boost::mutex  m_Mutex;
};

#endif // _TimerView_H_
