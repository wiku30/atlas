#include "kdtree.h"
#include "atlas.h"
#include <string>

int main()
{
	kdt tree[10];
	atlas at;
	for (int i = 1; i <= 5; i++)
	{
		string str1 = "tiles/map_";
		str1 += ('0' + i);
		str1 += ".tile";
		cout << str1 << endl;
		//ifstream ifs(str1.c_str());
		ifstream ifs(str1.c_str());
		tree[i].init(ifs, 9);
		
		string str2 = "trees/map_";
		str2 += ('0' + i);
		str2 += ".tree";
		ofstream ofs(str2);
		tree[i].save(ofs);
		at.add(&tree[i]);
	}

	ifstream ifs("trace/trace.txt");

	zone z, zold;
	for(int i=0;i<500;i++)
	{
		double x, y;
		zold = z;
		ifs >> x;
		if (ifs.eof())
		{
			break;
		}
		ifs >> y;
		at.query(z, x, y);

		z = update(zold, z);
		print(z);


	}



	/*
	ifstream inf("map1.txt");
	kdt tree, tree1;
	tree.init(inf, 8);

	int err_sum = 0;
	int sum = 0;
	ofstream ouf("tree.txt");
	tree.save(ouf);
	ifstream inf2("tree.txt");
	tree1.load(inf2);


	for (int i = -2048; i < 2048; i++)
	{
		for (int j = -2048; j < 2048; j++)
		{
			//cout << tree.exist(i, j) << endl;
			int tmp = tree1.exist(i, j);
			sum += tmp;
		}
	}
	cout << tree1.num_keys() << endl;
	*/

}