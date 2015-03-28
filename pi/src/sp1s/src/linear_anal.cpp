#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string.h>
#include <fstream>
#include <vector>
#include <iostream>
//#include "util/linear.h"
#include "util/vectorpairfile.h"
#include "boost/shared_ptr.hpp"

/*

void trim(std::string& str)
{
	string::size_type pos = str.find_last_not_of(' ');
	if(pos != string::npos)
	{
		str.erase(pos + 1);
		pos = str.find_first_not_of(' ');
		if(pos != string::npos)
			str.erase(0, pos);
	}
	else
		str.erase(str.begin(), str.end()); 
}

void split(std::string& s, std::string& delim,std::vector< std::string >* ret)
{
	size_t last = 0;
	size_t index=s.find_first_of(delim,last);
	while (index!=std::string::npos)
	{
		ret->push_back(s.substr(last,index-last));
		last=index+1;
		last=s.find_first_not_of(delim,last);		
		index=s.find_first_of(delim,last);
		
	}
	if (index-last>0)
	{
		ret->push_back(s.substr(last,index-last));
	}
}*/

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
/*
int main(int argc, char **argv)
{
    if(argc == 1 || strcmp(argv[1], "-h")==0)
    {
	printf("please run 'linear_anal  filename'  \n");
	return 0;
    }

    char * pFilePath = argv[1];    
    std::ifstream infile(pFilePath);
    if (!infile.is_open())
    {
	printf(" failed to open '%s'\n",  pFilePath);
	return 0;
    }
    std::string line;
    std::string delim = " ";
    int cols = 0;
    int rows = 0;
    double data;
    std::vector< double* > DataRows;
    while (std::getline(infile, line))
    {
	std::vector< std::string > dataVector;
	trim(line);
	split(line, delim, &dataVector);
        int c = (int) dataVector.size();
        if(cols > 0 && cols != c)
	{
		printf(" data count in each row are not same!");
		return 0;
	}
	cols = c;
	double * row = new double[c];
	int iCol = 0;
	for (std::vector<std::string>::iterator it = dataVector.begin() ; it != dataVector.end(); ++it)
	{
		std::istringstream iss(*it);
		if (!(iss >> data))
		{
			std::cout<<" failed to parse data "<<*it<< "  in row " << rows <<std::endl;
			return 0;
		}
		row[iCol++] = data;
	}
	DataRows.push_back(row);
	rows++;
    }
	
    double * X = new double[rows];
    double * Y = new double[rows];
    for(int round = 0; round < cols; round+=2)
    {
	    int i = 0;
	    for(std::vector<double*>::iterator it = DataRows.begin() ; it != DataRows.end(); ++it)
	    {
		double * row = *it;
		X[i] = row[round];
		Y[i] = row[round + 1];
		
		printf("	%f	%f\n", X[i], Y[i]);
		i++;		
	    }

	    double dFactor, dOffset, variance;
	    computeFactorOffset(X,Y,rows, dFactor, dOffset, variance);
	    printf("Factor: %f\n",dFactor);
	    printf("Offset: %f\n",dOffset);
	    printf("variance: %f\n",variance);
	    printf("________________________________________________________________________\n");
    }
    delete[] X;
    delete[] Y;

    for(std::vector<double*>::iterator it = DataRows.begin() ; it != DataRows.end(); ++it)
    {
	delete[] *it;
    }
}*/
