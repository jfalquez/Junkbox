#ifndef IMAGEHELPERS_H
#define IMAGEHELPERS_H


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// adjust mean and variance of Image1 brightness to be closer to Image2
inline void BrightnessCorrectionImagePair(
        unsigned char *pData1,                  //< Input: Pointer 1
        unsigned char *pData2,                  //< Input: Pointer 2
        int nImageSize                          //< Input: Number of pixels in image
        )
{
    unsigned char* pData1_Orig = pData1;
    const int     nSampleStep = 1;
    int           nSamples    = 0;

     // compute mean
    float fMean1  = 0.0;
    float fMean2  = 0.0;
    float fMean12 = 0.0;
    float fMean22 = 0.0;

    for(int ii=0; ii<nImageSize; ii+=nSampleStep, pData1+=nSampleStep,pData2+=nSampleStep) {
        fMean1  += (*pData1);
        fMean12 += (*pData1) * (*pData1);
        fMean2  += (*pData2);
        fMean22 += (*pData2) * (*pData2);
        nSamples++;
    }

    fMean1  /= nSamples;
    fMean2  /= nSamples;
    fMean12 /= nSamples;
    fMean22 /= nSamples;

    // compute std
    float fStd1 = sqrt(fMean12 - fMean1*fMean1);
    float fStd2 = sqrt(fMean22 - fMean2*fMean2);

    // mean diff;
    //float mdiff = mean1 - mean2;
    // std factor
    float fRatio = fStd2/fStd1;
    // normalize image
    float tmp;
    // reset pointer
    pData1 = pData1_Orig;

    int nMean1 = (int)fMean1;
    int nMean2 = (int)fMean2;

    for(int ii=0; ii < nImageSize; ++ii) {

        tmp = (float)( pData1[ii] - nMean1 )*fRatio + nMean2;
        if(tmp < 0)  tmp = 0;
        if(tmp > 255) tmp = 255;
        pData1[ii] = (unsigned char)tmp;
    }

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SAD score image1 with image2 -- images are assumed to be same type & dimensions
// returns: SAD score
template < typename T >
inline float ScoreImages(
        const cv::Mat&              Image1,
        const cv::Mat&              Image2
        )
{
    float fScore = 0;
    for( int ii = 0; ii < Image1.rows; ii++ ) {
        for( int jj = 0; jj < Image1.cols; jj++ ) {
            fScore += fabs(Image1.at<T>(ii, jj) - Image2.at<T>(ii, jj));
        }
    }
    return fScore;
}


#endif // IMAGEHELPERS_H
