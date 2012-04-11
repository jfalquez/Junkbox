#ifndef _HDMI_H_
#define _HDMI_H_

#include "RPG/Devices/Camera/CameraDriverInterface.h"

#include "Blackmagic/DeckLinkAPI.h"


class HDMIDriver : public CameraDriver
{
    public:
        HDMIDriver();
        virtual ~HDMIDriver();
        bool Capture( std::vector<cv::Mat>& vImages );
        bool Init();
    private:
        unsigned int                    m_nNumImages;
        unsigned int                    m_nImageWidth;
        unsigned int					m_nImageHeight;

		IDeckLink* 						m_pDeckLink;
		IDeckLinkInput*					m_pDeckLinkInput;
		IDeckLinkDisplayModeIterator*	m_pDisplayModeIterator;
		IDeckLinkIterator*				m_pIterator;

};

#endif
