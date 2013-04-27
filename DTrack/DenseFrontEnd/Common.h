#ifndef _COMMON_H_
#define _COMMON_H_


const int   MAX_PYR_LEVELS = 5;


// TODO: this is retarded since we are not using them as flags.. no need to have them powers of 2.
// Eventually implement flags so that, say, tracking is good AND loop closure is flagged.
enum eTrackingState
{
    eTrackingGood           = 1,
    eTrackingPoor           = 2,
    eTrackingBad            = 4,
    eTrackingFail           = 8,
    eTrackingLoopClosure    = 16
};

#endif
