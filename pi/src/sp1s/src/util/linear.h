#pragma once
#include "math.h"
#include "stdio.h"
#include "../dbl_cmp.h"
#include <stdlib.h>  

// Y = X * dFactor + dOffset
void computeFactorOffset(double* arrX, double* arrY, long lSize,
		double& dFactor, double& dOffset, double& variance)
{
	// Compute the average dFactor from each pair of samples
	double* arrFactor = (double *) malloc( sizeof(double) *(lSize - 1) );
	double dSum = 0.0;
	double dStepFactor = 0.0;
	long length = 0;
	for(long i=1;i< lSize; i++)
	{
		if (DBLCMPEQ(arrY[i],arrY[i - 1])
			|| DBLCMPEQ(arrY[i - 1], 0))
			continue;
		double deltaY = arrY[i] - arrY[i-1];
		double deltaX = arrX[i] - arrX[i-1];
		dStepFactor = deltaY / deltaX;
		arrFactor[length] = dStepFactor;
		dSum += dStepFactor;
		length++;
	}
    double dAvg = dSum / length;
    dFactor = dAvg;
    dOffset = arrY[lSize - 1] - arrX[lSize - 1]*dFactor;

    // compute variance
    variance = 0.0;
    for(long i=1;i< lSize; i++)
    {
	if (DBLCMPEQ(arrX[i], 0))
    		continue;
    	double dComputeY = arrX[i]*dFactor + dOffset;
    	variance += pow(dComputeY - arrY[i], 2 );
    }
    free(arrFactor);
}

