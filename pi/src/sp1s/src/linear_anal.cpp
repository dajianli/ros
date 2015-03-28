#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string.h>
#include <fstream>
#include <vector>
#include <iostream>
#include "util/linear.h"
#include "util/vectorpairfile.h"
#include "boost/shared_ptr.hpp"

int main(int argc, char **argv)
{
    if(argc == 1 || strcmp(argv[1], "-h")==0)
    {
    	printf("please run 'linear_anal  filename'  \n");
    	return 0;
    }

    char * pFilePath = argv[1];
    vectorpairfile pairfile;
    pairfile.loadfile(pFilePath);
    for(int round = 0 ; round < pairfile.PairCount(); round++)
    {
	boost::shared_ptr<vectorPair> pair = pairfile[round];
	for(int i = 0 ; i < pair->size(); i++)
	{
		printf("	%f	%f\n", pair->X[i], pair->Y[i]);
	}
	printf("Factor: %f\n",pair->m_dFactor);
	printf("Offset: %f\n",pair->m_dOffset);
	printf("variance: %f\n",pair->m_variance);
	printf("________________________________________________________________________\n");
    }
}
