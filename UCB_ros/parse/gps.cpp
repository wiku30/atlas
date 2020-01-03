#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <sstream>
using namespace std;

void parse(ifstream& inf)
{
    string tmp;
	long long time1, time2, time0;
	double xx, yy, zz;
	while (!inf.eof())
	{
		inf >> tmp;
		//if (tmp == "gps_week:")
		if (tmp == "secs:")
		{
			inf >> time1 >> tmp >> time2;
			//time0 = time1 * 86400 * 1000 + time2;
			time0 = time1 * 1000000 * 1000 + time2;
			cout << time0 << " ";
		}
		if (tmp == "latitude:")
		{
			inf >> xx >> tmp >> yy >> tmp >> zz;
			cout << setw(18) << fixed << setprecision(9) << xx << " " << yy << " " << zz << endl;
		}
	}
}

int main()
{
    string name;
    cin>>name;

    stringstream ss;
    ss<<name<<".txt";
    string path=ss.str();
    ifstream inf(path);
    if(inf)
    {
        parse(inf);
    }
}