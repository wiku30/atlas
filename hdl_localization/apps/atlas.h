#pragma once


#include <set>
#include <vector>
#include <deque>
#include "kdtree.h"
#include "conversion.h"
#include <cassert>



using namespace std;

typedef map<int, kdt*> zone;

vec2 std_conv(double x, double y);


struct submap //not finished
{
	deque<int> now_ids;
	int update(zone in)
	{
		if (in.size() <= 3)
		{
			int to_update = 0;
			for (auto i : in)
			{
				int id = i.first;
				to_update = 1;
				for (auto now_id : now_ids)
				{
					if (now_id == id)
					{
						to_update = 0;
					}
				}
				if (to_update)
				{
					now_ids.push_front(id);
					if (now_ids.size() >= 4)
					{
						now_ids.pop_back();
					}
					return 1;
				}
			}
		}
		else
		{
			for (auto i : in)
			{

			}
			assert(0);
		}
		return 0;
	}
	int size() { return now_ids.size(); }
	int operator[](int x) { return now_ids[x]; }
	void print()
	{
		int sz = size();
		for (int x = 0; x < sz; x++)
		{
			cout << now_ids[x]+1 << " ";
		}
		cout << endl;
	}
};


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

zone update(const zone& old, const zone& neww, int& ifchanged);

void print(const zone& a);

void atlas::query(zone& output, double x, double y, int verb)
{
	vec2 tile = (*conversion)(x, y);
	if (verb)
		cout << "tile coordinate:" << tile.x << " " << tile.y << endl;
	output.clear();
	for (int ii = 0; ii < map_bases.size(); ii++)
	{
		auto i = map_bases[ii];
		if (i->exist(tile))
		{
			output[ii] = i;
			if (verb)
			{
				cout << ii + 1 << " ";
			}
		}
	}
	if (verb)
	{
		cout << endl;
	}
}

vec2 std_conv(double x, double y)
{
	vec2 res;
	res.x = int(x + 2.5) / 5;
	res.y = int(y + 2.5) / 5;
	return res;
}

int issubsetof(const zone& z1, const zone& z2)
{
	for (auto it : z1)
	{

		if (z2.find(it.first) == z2.end())
		{
			return 0;
		}
	}
	return 1;
}

zone update(const zone& old, const zone& neww, int& ifchanged)
{
	if (issubsetof(neww, old))
	{
		ifchanged = 0;
		return old;
	}
	else
	{
		ifchanged = 1;
		return neww;
	}
}

void print(const zone& a)
{
	for (auto it : a)
	{
		cout << it.first + 1 << " ";
	}
	cout << endl;
}