#pragma once
#ifndef CONV
#define CONV

#include "kdtree.h"

class conv
{
public:
	int ifprev = 0;
	conv* prev = NULL;
	virtual vec2 process(double, double) = 0;
	virtual vec2 process(vec2 in)
	{
		return process(in.x, in.y);
	}
	virtual vec2 operator () (double x, double y)
	{
		return (*this)(vec2(x, y));
	}
	virtual vec2 operator () (vec2 in)
	{
		if (ifprev)
		{
			return process((*prev)(in));
		}
		else
		{
			return process(in);
		}
	}
	/*
	virtual conv operator *(conv& in)
	{

	}
	*/
};

class grid_map: public conv
{
public:
	double x_ori=0;
	double y_ori=0;
	vec2 process (double x, double y)
	{
		return vec2(int(x - x_ori + 2.5) / 5, int(y - y_ori + 2.5) / 5);
	}
};

static grid_map gridmap;

#endif