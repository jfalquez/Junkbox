#include <stdio.h>

#include "CaptureDelegate.h"

#include "Blackmagic/DeckLinkAPI.h"


// mutex stuff.. I don't like it much, perhaps use boost?
pthread_mutex_t					sleepMutex;
pthread_cond_t					sleepCond;



int main(int argc, char *argv[])
{
	int							exitStatus = 1;

	HRESULT						result;

	IDeckLinkIterator*			pDeckLinkIterator;
	IDeckLinkConfiguration*		pDeckLinkConfig;
	IDeckLink*					pDeckLink;
	IDeckLinkInput*				pDeckLinkInput;
	BMDVideoInputFlags			inputFlags = 0;
	BMDDisplayMode				selectedDisplayMode = bmdModeHD720p60;
	BMDPixelFormat				pixelFormat = bmdFormat8BitYUV;

	CaptureDelegate*			pDelegate;

	// instantiate an iterator
	pDeckLinkIterator = CreateDeckLinkIteratorInstance();

	if( !pDeckLinkIterator ) {
		fprintf(stderr, "This application requires the DeckLink drivers installed.\n");
		goto bail;
	}

	// Connect to the first DeckLink instance
	result = pDeckLinkIterator->Next( &pDeckLink );
	if( result != S_OK ) {
		fprintf(stderr, "No DeckLink PCI cards found.\n");
		goto bail;
	}

	// query the interface
	if( pDeckLink->QueryInterface(IID_IDeckLinkInput, (void**)&pDeckLinkInput) != S_OK ) {
		goto bail;
	}

	// set up callback delegate
	pDelegate = new CaptureDelegate();
	pDeckLinkInput->SetCallback( pDelegate );

	// set up video input mode
    result = pDeckLinkInput->EnableVideoInput( selectedDisplayMode, pixelFormat, inputFlags );
    if( result != S_OK ) {
		fprintf(stderr, "Failed to enable video input. Is another application using the card?\n");
        goto bail;
    }

	// start stream
	result = pDeckLinkInput->StartStreams();
    if(result != S_OK) {
        goto bail;
    }

	// All Okay.
	exitStatus = 0;
	fprintf(stderr, "Program running...\n");

	// Block main thread until signal occurs
	pthread_mutex_lock(&sleepMutex);
	pthread_cond_wait(&sleepCond, &sleepMutex);
	pthread_mutex_unlock(&sleepMutex);
	fprintf(stderr, "Stopping capture. ");


bail:

	fprintf(stderr, "Closing devices... ");

    if( pDeckLinkInput != NULL ) {
        pDeckLinkInput->Release();
        pDeckLinkInput = NULL;
    }
    if( pDeckLink != NULL ) {
        pDeckLink->Release();
        pDeckLink = NULL;
    }
	if( pDeckLinkIterator != NULL ) {
		pDeckLinkIterator->Release();
	}

	fprintf(stderr, "Done.\n");
	return exitStatus;
}

