/*
 * File:   CaptureDelegate.cpp
 * Author: jmf
 *
 * Created on April 10, 2012, 4:25 PM
 */

#include "CaptureDelegate.h"

#include <stdio.h>
#include <stdlib.h>

extern pthread_cond_t sleepCond;


CaptureDelegate::CaptureDelegate() : m_refCount(0), m_frameCount(0), m_maxFrames(-1), m_timecodeFormat(0)
{
	pthread_mutex_init( &m_mutex, NULL );
	m_pContext = new zmq::context_t( 1 );
	m_pSocket = new zmq::socket_t( *m_pContext, ZMQ_PUB );
	m_pSocket->bind("tcp://*:6666");
}

CaptureDelegate::~CaptureDelegate()
{
	pthread_mutex_destroy(&m_mutex);
}

ULONG CaptureDelegate::AddRef(void)
{
	pthread_mutex_lock(&m_mutex);
		m_refCount++;
	pthread_mutex_unlock(&m_mutex);

	return (ULONG)m_refCount;
}

ULONG CaptureDelegate::Release(void)
{
	pthread_mutex_lock(&m_mutex);
		m_refCount--;
	pthread_mutex_unlock(&m_mutex);

	if (m_refCount == 0)
	{
		delete this;
		return 0;
	}

	return (ULONG)m_refCount;
}

HRESULT CaptureDelegate::VideoInputFrameArrived(IDeckLinkVideoInputFrame* videoFrame, IDeckLinkAudioInputPacket* /* audioFrame */)
{
	void*					frameBytes;

	// Handle Video Frame
	if(videoFrame)
	{

		if (videoFrame->GetFlags() & bmdFrameHasNoInputSource) {
			fprintf(stderr, "Frame received (#%lu) - No input signal detected\n", m_frameCount);
		}
		else {
			const char *timecodeString = NULL;
			if( m_timecodeFormat != 0 ) {
				IDeckLinkTimecode *timecode;
				if (videoFrame->GetTimecode(m_timecodeFormat, &timecode) == S_OK)
				{
					timecode->GetString(&timecodeString);
				}
			}

			/**/
			fprintf(stderr, "Frame received (#%lu) [%s] - Valid Frame - Height: %li - Size: %li bytes\n",
				m_frameCount,
				timecodeString != NULL ? timecodeString : "No timecode",
			    videoFrame->GetHeight(),
				videoFrame->GetRowBytes() * videoFrame->GetHeight());
			/**/

			if( timecodeString ) {
				free((void*)timecodeString);
			}

			// publish
			zmq::message_t ZmqMsg(4 + 12 + videoFrame->GetRowBytes() * videoFrame->GetHeight());
			// push number of images in message
			int NUM_IMAGES = 1;
			int WIDTH = 1280;
			int HEIGHT = 720;
			int RGB_CHAN = 1;
			int IMG_TYPE = 0; // Or 24?
			char* MsgPtr = (char*)ZmqMsg.data();
			memcpy( MsgPtr, &NUM_IMAGES, sizeof(NUM_IMAGES) );
			MsgPtr += sizeof(NUM_IMAGES);
			memcpy( MsgPtr, &WIDTH, sizeof(WIDTH) );
			MsgPtr += sizeof(WIDTH);
			memcpy( MsgPtr, &HEIGHT, sizeof(HEIGHT) );
			MsgPtr += sizeof(HEIGHT);
			memcpy( MsgPtr, &IMG_TYPE, sizeof(IMG_TYPE) );
			MsgPtr += sizeof(IMG_TYPE);

			videoFrame->GetBytes(&frameBytes);
		    memcpy( MsgPtr, frameBytes, videoFrame->GetRowBytes() * videoFrame->GetHeight() );
			m_pSocket->send( ZmqMsg );
			if( 0/* m_pSocket */ ) {
				// this is hardcoded for now
				int NUM_IMAGES = 1;

				int WIDTH = 640;
				int HEIGHT = 480;
				int RGB_CHAN = 1;
				int IMG_TYPE = 0; // Or 24?

				int IMG_SIZE = WIDTH * HEIGHT * RGB_CHAN;

				zmq::message_t ZmqMsg(4 + (NUM_IMAGES * (12 + WIDTH * HEIGHT * RGB_CHAN)));
				char* MsgPtr = (char*)ZmqMsg.data();

				// push number of images in message
				memcpy( MsgPtr, &NUM_IMAGES, sizeof(NUM_IMAGES) );
				MsgPtr += sizeof(NUM_IMAGES);


				// get bytes
//				frameBytes = (void*)MsgPtr;
				videoFrame->GetBytes(&frameBytes);


				// push first image
				memcpy( MsgPtr, &WIDTH, sizeof(WIDTH) );
				MsgPtr += sizeof(WIDTH);
				memcpy( MsgPtr, &HEIGHT, sizeof(HEIGHT) );
				MsgPtr += sizeof(HEIGHT);
				memcpy( MsgPtr, &IMG_TYPE, sizeof(IMG_TYPE) );
				MsgPtr += sizeof(IMG_TYPE);
				char* FbPtr = (char*)frameBytes;
				int padding = videoFrame->GetRowBytes() - 1280;
				for( int jj = 0; jj < HEIGHT; jj++ ) {
					for( int ii = 0; ii < WIDTH; ii++ ) {
						*MsgPtr = *(FbPtr);
						MsgPtr++;
						FbPtr += 2;
					}
					FbPtr += padding;
				}
//				memcpy( MsgPtr, frameBytes, IMG_SIZE );
//				MsgPtr += IMG_SIZE;

				// push second image
//				memcpy( MsgPtr, &WIDTH, sizeof(WIDTH) );
//				MsgPtr += sizeof(WIDTH);
//				memcpy( MsgPtr, &HEIGHT, sizeof(HEIGHT) );
//				MsgPtr += sizeof(HEIGHT);
//				memcpy( MsgPtr, &IMG_TYPE, sizeof(IMG_TYPE) );
//				MsgPtr += sizeof(IMG_TYPE);
//				for( int ii = 0; ii < IMG_SIZE; ii++ ) {
//					*(MsgPtr++) = *((char*)frameBytes+IMG_SIZE+ii);
//				}
//				memcpy( MsgPtr, frameBytes, IMG_SIZE );

				m_pSocket->send( ZmqMsg );
			}
		}

		m_frameCount++;

		if (m_maxFrames > 0 && m_frameCount >= m_maxFrames)
		{
			pthread_cond_signal(&sleepCond);
		}
	}

    return S_OK;
}

HRESULT CaptureDelegate::VideoInputFormatChanged(BMDVideoInputFormatChangedEvents events, IDeckLinkDisplayMode *mode, BMDDetectedVideoInputFormatFlags)
{
    return S_OK;
}
