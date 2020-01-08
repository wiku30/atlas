#pragma once

#include <iostream>
#include <fstream>
#include <set>
#include <unordered_map>
#include <map>
#include <vector>
#include <cassert>
using namespace std;


struct vec2
{
	double x;
	double y;
	vec2() :x(0), y(0) {}
	vec2(double xx, double yy) :x(xx), y(yy) {}
	bool operator < (const vec2& a) const
	{
		return x < a.x || (x == a.x && y < a.y);
	}
};

typedef map<vec2, int> keyset;

class kdt
{
	int levels;
	int offsetx;
	int offsety;
	int range;
	int max_grid_level;
	vector < keyset > maps;
	int checkmap(int lv, int x, int y);
	int num_tiles;
	int perform_count;
	int xxmin, xxmax, yymin, yymax;
public:
	void init(ifstream&, int);
	void init(ifstream&, int, int, int);
	void optimize(int);
	void optimize();
	int exist(int x, int y);
	int exist(vec2 xy);
	void printcount() { cout << perform_count << endl; perform_count = 0; }
	void save(ofstream& ofs);
	void load(ifstream& ifs);
	int num_keys();
};




