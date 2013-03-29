/*
 * File:   GLVarHistory.h
 * Author: alonso
 *
 * Created on June 18, 2012, 9:53 AM
 */

#ifndef _GL_VARHISTORY_H_
#define	_GL_VARHISTORY_H_


#include <string>
#include <deque>
#include <vector>
#include <sstream>
#include <iomanip>

#include <SceneGraph/SceneGraph.h>

#include "GLSimpleObjects.h"


class GLVarHistory
{
    public:

        ////////////////////////////////////////////////////////////////////////
        GLVarHistory(){}

        ////////////////////////////////////////////////////////////////////////
        ~GLVarHistory(){}

        ////////////////////////////////////////////////////////////////////////
        void InitReset(std::string sVarName, unsigned int nMaxLength)
        {
            m_sVarName          = sVarName;
            m_nMaxHistoryLength = nMaxLength;

            m_vHistory.clear();
        }

        ////////////////////////////////////////////////////////////////////////
        void Update(double dVarValue)
        {
            m_vHistory.push_back(dVarValue);

            if(m_vHistory.size() > m_nMaxHistoryLength)
                m_vHistory.pop_front();

            std::deque<double> orderedHistory = m_vHistory;
            std::sort(orderedHistory.begin(), orderedHistory.end() );
            m_dMedian = orderedHistory[ orderedHistory.size()/2 ];
        }

        ////////////////////////////////////////////////////////////////////////
        void Draw(
                   const float fWidth,
                   const float fHeight,
                   const float fOffsetX,
                   const float fOffsetY,
                   std::vector<double>& vfColor
                 )
        {
             // draw history
            int   nSteps  = (int)m_vHistory.size();
            float fTop    = fOffsetY;
            float fBottom = fOffsetY + fHeight;
            float fLeft   = fOffsetX;
            float fRight  = fOffsetX + fWidth;
            float fStepX  = fWidth / ( m_nMaxHistoryLength - 1 );
            float fXStart = fWidth - ( nSteps - 1 ) * fStepX;
            float fScale  = 0.0;

            if(nSteps > 1)
            {
                for(int ii=0; ii < nSteps; ii++) {
                    if(m_vHistory[ii] > fScale) {
                        fScale = m_vHistory[ii];
                    }
                }

                if(fScale > 0.0) {
                    fScale = fHeight/fScale;
                }

                glColor4f( vfColor[0], vfColor[1], vfColor[2],0.6);
                for(int ii=0; ii < nSteps-1; ii++)
                {
                    glBegin(GL_QUADS);
                    glVertex2f( fXStart + ii * fStepX,    fBottom );
                    glVertex2f( fXStart + ii * fStepX,    fBottom - m_vHistory[ii]*fScale );
                    glVertex2f( fXStart + (ii+1) * fStepX,fBottom - m_vHistory[ii+1]*fScale );
                    glVertex2f( fXStart + (ii+1) * fStepX,fBottom );
                    glEnd();
                }
            }

            // draw var name and last value
            glColor4f(1.0,1.0,1.0,1.0);
            std::stringstream ss;
            std::string sLine;
            float fDataOffsetX = 2.0;

            ss << m_sVarName;
            if(nSteps > 0)
                ss  << " : " << std::setprecision(4) << m_vHistory.back();

            sLine = ss.str();
            //fDataOffsetX = (fWidth - (float)gl_width( sLine.c_str() ) )/2.0;
            glRasterPos2f(fLeft + fDataOffsetX, fBottom - 2.0);
            m_glText.Draw( sLine );

            glRasterPos2f(fLeft + fWidth/2, fBottom - 2.0);
            ss.str("");
            ss << " median : ";
            if(nSteps > 0)
                ss << m_dMedian;
            sLine = ss.str();
            m_glText.Draw( sLine );

            // draw median line
            glColor4f( 229.0/255.0, 104.0/255.0, 63.0/255.0, 1.0);
            float fScaledMedian = (float)(fScale*m_dMedian);
            drawLine( fLeft, fBottom - fScaledMedian, fRight, fBottom - fScaledMedian);

            // draw globject bounding box
            glLineWidth(1);
            glColor4f(1.0,1.0,1.0,1.0);
            glBegin(GL_LINE_LOOP);
                glVertex2f( fLeft,  fTop );
                glVertex2f( fRight, fTop );
                glVertex2f( fRight, fBottom );
                glVertex2f( fLeft,  fBottom );
            glEnd();
        }


    private:

        std::string         m_sVarName;
        unsigned int        m_nMaxHistoryLength;
        double              m_dMedian;
        std::deque<double>  m_vHistory;
        SceneGraph::GLText  m_glText;

};


#endif	/* GLVARHISTORY_H */

