#pragma once


#include <set>
#include <vector>
#include <deque>
#include "kdtree.h"
#include "conversion.h"



using namespace std;

typedef map<int, kdt*> zone;

vec2 std_conv(double x, double y);



class atlas
{
public:
	vector<kdt*> map_bases;
	deque<vec2> path;
	//vec2(*conversion)(double x, double y); //from x,y to grids

	conv* conversion;

	void query(zone& output, double x, double y, int verb = 0);
	void add(kdt* m)
	{
		map_bases.push_back(m);
	}
	atlas()
	{
		conversion = &(gridmap);
	}
};

zone update(const zone& old, const zone& neww);

void print(const zone& a);