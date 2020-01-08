#include "atlas.h"

void atlas::query(zone& output, double x, double y, int verb)
{
	vec2 tile = (*conversion)(x, y);
	output.clear();
	for (int ii=0;ii<map_bases.size();ii++)
	{
		auto i = map_bases[ii];
		if (i->exist(tile))
		{
			output[ii] = i;
			if (verb)
			{
				cout << ii+1 << " ";
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

zone update(const zone& old, const zone& neww)
{
	if (issubsetof(neww, old))
	{
		return old;
	}
	else
	{
		return neww;
	}
}

void print(const zone& a)
{
	for (auto it : a)
	{
		cout << it.first+1 << " ";
	}
	cout << endl;
}

