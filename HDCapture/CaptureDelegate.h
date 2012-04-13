/*
 * File:   CaptureDelegate.h
 * Author: jmf
 *
 * Created on April 10, 2012, 4:25 PM
 */

#ifndef CAPTUREDELEGATE_H
#define	CAPTUREDELEGATE_H

#include <pthread.h>
#include <zmq.hpp>

#include "DeckLinkAPI.h"


class CaptureDelegate :  public IDeckLinkInputCallback {
public:
	CaptureDelegate();
	~CaptureDelegate();

	virtual HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, LPVOID *ppv) { return E_NOINTERFACE; }
	virtual ULONG STDMETHODCALLTYPE AddRef(void);
	virtual ULONG STDMETHODCALLTYPE  Release(void);
	virtual HRESULT STDMETHODCALLTYPE VideoInputFormatChanged(BMDVideoInputFormatChangedEvents, IDeckLinkDisplayMode*, BMDDetectedVideoInputFormatFlags);
	virtual HRESULT STDMETHODCALLTYPE VideoInputFrameArrived(IDeckLinkVideoInputFrame*, IDeckLinkAudioInputPacket*);

private:
	zmq::context_t*				m_pContext;
	zmq::socket_t*				m_pSocket;

	unsigned long				m_frameCount;
	int							m_maxFrames;
	BMDTimecodeFormat			m_timecodeFormat;
	ULONG						m_refCount;
	pthread_mutex_t				m_mutex;

} ;

#endif	/* CAPTUREDELEGATE_H */

