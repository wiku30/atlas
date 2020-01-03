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

void kdt::init(ifstream& inf, int levels_)
{
	int os = -(1 << (levels_ - 1));
	init(inf, levels_, os, os);
}

void kdt::init(ifstream& inf, int levels_, int offsetx_, int offsety_)
{
	levels = levels_;
	range = 1 << levels;
	offsetx = offsetx_;
	offsety = offsety_;
	perform_count = 0;
	max_grid_level = 0;
	num_tiles = 0;
	for (int i = 0; i <= levels; i++)
	{
		maps.push_back(keyset());
	}

	xxmin = yymin = 1 << 30;
	xxmax = yymax = -xxmin;

	while (1)
	{
		double x, y;
		inf >> x;
		if (inf.eof())
			break;
		inf >> y;
		num_tiles++;
		int xx = int(x) - offsetx;
		int yy = int(y) - offsety;
		if (xx > xxmax)
			xxmax = xx;
		if (xx < xxmin)
			xxmin = xx;

		if (yy > yymax)
			yymax = yy;
		if (yy < yymin)
			yymin = yy;

		assert(xx >= 0 && yy >= 0 && xx < range && yy < range);
		maps[0][vec2(xx, yy)] = 1;
	}
	optimize();
}

int kdt::exist(vec2 xy)
{
	return exist(xy.x, xy.y);
}

int kdt::exist(int x, int y)
{
	int xx, yy;
	for (int i = max_grid_level; i >= 0; i--)
	{
		xx = x - offsetx;
		yy = y - offsety;
		if (xx<xxmin || xx>xxmax || yy<yymin || yy>yymax)
		{
			return 0;
		}
		xx = (xx >> i << i);
		yy = (yy >> i << i);
		int res = checkmap(i, xx, yy);
		if (res >= 0)
			return res;
	}
	//cout << "ERROR!" << endl;
	return 0;
}

int kdt::checkmap(int lv, int x, int y)
{

	perform_count++;
	if (maps[lv].find(vec2(x, y)) == maps[lv].end())
	{
		return lv ? -1 : 0;
	}
	return maps[lv][vec2(x, y)];
}

void kdt::optimize(int lv)
{
	int step = 1 << lv;
	int imin = xxmin / (2 * step) * 2 * step;
	int imax = xxmax / step * step;
	int jmin = yymin / (2 * step) * 2 * step;
	int jmax = yymax / step * step;
	for (int i = imin; i <= imax; i += 2 * step)
	{
		for (int j = jmin; j <= jmax; j += 2 * step)
		{
			if (checkmap(lv, i, j) == 1 &&
				checkmap(lv, i, j + step) == 1 &&
				checkmap(lv, i + step, j) == 1 &&
				checkmap(lv, i + step, j + step) == 1)
			{
				maps[lv].erase(vec2(i, j));
				maps[lv].erase(vec2(i, j + step));
				maps[lv].erase(vec2(i + step, j));
				maps[lv].erase(vec2(i + step, j + step));
				maps[lv + 1][vec2(i, j)] = 1;
			}
			else if (checkmap(lv, i, j) == 0 &&
				checkmap(lv, i, j + step) == 0 &&
				checkmap(lv, i + step, j) == 0 &&
				checkmap(lv, i + step, j + step) == 0)
			{
				maps[lv].erase(vec2(i, j));
				maps[lv].erase(vec2(i, j + step));
				maps[lv].erase(vec2(i + step, j));
				maps[lv].erase(vec2(i + step, j + step));
				maps[lv + 1][vec2(i, j)] = 0;
			}
		}
	}
}

int kdt::num_keys()
{
	int res = 0;
	for (auto i : maps)
	{
		res += i.size();
	}
	return res;
}

void kdt::optimize()
{
	for (int i = 0; i < levels; i++)
	{
		optimize(i);
		if (maps[i + 1].size())
		{
			max_grid_level = i + 1;
		}
	}
}

void write_num(ofstream& ouf, int num)
{
	//ouf.write((char*)&num, sizeof(int));
	ouf << num << " ";
}

void kdt::load(ifstream& ifs)
{
	int tmp;
	ifs >> levels >> max_grid_level >> offsetx >> offsety;
	cout << levels << " " << max_grid_level << " " << offsetx << " " << offsety << endl;
	ifs >> xxmin >> xxmax >> yymin >> yymax;
	ifs >> tmp >> tmp;
	for (int i = 0; i <= max_grid_level; i++)
	{
		int sz;
		ifs >> tmp >> sz;
		keyset mp;
		for (int j = 0; j < sz; j++)
		{
			int x, y, type;
			ifs >> x >> y >> type;
			mp[vec2(x, y)] = type;
		}

		maps.push_back(mp);
	}
}

void kdt::save(ofstream& ouf)
{
	write_num(ouf, levels);
	write_num(ouf, max_grid_level);
	write_num(ouf, offsetx);
	write_num(ouf, offsety);
	ouf << endl;
	write_num(ouf, xxmin);
	write_num(ouf, xxmax);
	write_num(ouf, yymin);
	write_num(ouf, yymax);
	ouf << endl;
	write_num(ouf, num_tiles);
	write_num(ouf, num_keys());
	ouf << endl;
	for (int i = 0; i <= max_grid_level; i++)
	{
		ouf << endl;
		write_num(ouf, i);
		write_num(ouf, maps[i].size());
		ouf << endl;
		for (auto it : maps[i])
		{
			write_num(ouf, it.first.x);
			write_num(ouf, it.first.y);
			write_num(ouf, it.second);
			ouf << endl;
		}
	}
}
