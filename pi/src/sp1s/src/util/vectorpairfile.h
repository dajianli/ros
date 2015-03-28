#pragma once

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string.h>
#include <fstream>
#include <vector>
#include <iostream>
#include "boost/shared_ptr.hpp"
#include "linear.h"

class vectorPair
{
public:
	vectorPair(int capacity = 0)
	{
		X = Y = err = NULL;
		if(capacity > 0 )
			Reset(capacity);
	}

	void Reset(int capacity)
	{
		Release();
		X = new double[capacity];
		Y = new double[capacity];
		err = new double[capacity];
		m_capacity = capacity;
	}
	void compute()
	{
		computeFactorOffset(X,Y,m_capacity, m_dFactor, m_dOffset, m_variance);
	}
	
	double X2Y(double x)
	{
		return m_dFactor * x + m_dOffset;
	}

	double Y2X(double y)
	{
		return ( y - m_dOffset ) / m_dFactor;
	}

	double InterpolateY2X(double y)
	{
		if(m_capacity<2)
			return 0;
		int p1 = 0;
		int p2 = 1;
		for(int i=0;i<m_capacity;i++)
		{
			if(DBLCMPEQ(Y[i], y))
				return X[i];
			if(DBLCMPGT(Y[i], y))
				break;
			if(i + 1 == m_capacity)
				break;

			p1 = i;
			p2 = i + 1;
			if(ISBETWEEN(y,Y[p1],Y[p2]))
				break;
		}
		double rate = ( X[p2] - X[p1] ) / ( Y[p2] - Y[p1] );
		return X[p1] + (y - Y[p1]) * rate;
	}

	~vectorPair()
	{
		Release();
	}
	int size() { return m_capacity; }
	double *X, *Y, *err;	
	double m_dFactor;
	double m_dOffset;
	double m_variance;
private:
	void Release()
	{
		if(X != NULL)
			delete[] X;
		if(Y != NULL)
			delete[] Y;
		if(err != NULL)
			delete[] err;
		X = Y = err = NULL;
		m_capacity = 0;
	}
	int m_capacity;
};

class vectorpairfile
{
public:
	vectorpairfile()
	{
	    m_rowCount = 0;
	    m_colCount = 0;
	}

	bool loadfile(char * pFilePath )
	{
	    std::ifstream infile(pFilePath);
    	    if (!infile.is_open())
	    {
		printf(" failed to open '%s'\n",  pFilePath);
		return false;
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
			return false;
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
				return false;
			}
			row[iCol++] = data;
		}
		DataRows.push_back(row);
		rows++;
	    }
	
	    for(int round = 0; round < cols; round+=2)
	    {
		    boost::shared_ptr<vectorPair> pair(new vectorPair());
		    pair->Reset(rows);
		    int i = 0;
		    for(std::vector<double*>::iterator it = DataRows.begin() ; it != DataRows.end(); ++it)
		    {
			double * row = *it;
			pair->X[i] = row[round];
			pair->Y[i] = row[round + 1];		
			i++;
		    }
		    pair->compute();
		    m_Pairs.push_back(pair);
	    }

	    for(std::vector<double*>::iterator it = DataRows.begin() ; it != DataRows.end(); ++it)
    	    {
		delete[] *it;
    	    }
	    m_colCount = cols;
	    m_rowCount = rows;
	    return true;
	}

	int PairCount() { return (int)m_Pairs.size(); }
	boost::shared_ptr<vectorPair> operator [] (int index)
	{
		return m_Pairs[index];
	}
	
private:
	void trim(std::string& str)
	{
		std::string::size_type pos = str.find_last_not_of(' ');
		if(pos != std::string::npos)
		{
			str.erase(pos + 1);
			pos = str.find_first_not_of(' ');
			if(pos != std::string::npos)
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
	}
	int m_rowCount;
	int m_colCount;
	std::vector<boost::shared_ptr<vectorPair> >	m_Pairs;
	
};
