#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <sstream>
using namespace std;

void parse(ifstream& inf)
{
    string foo;
    long long time1,time2,time;
    double xyz[3];
    inf>>foo>>time1>>time2>>foo;
    time=time1*1000+time2/1e+6;
    cout<<time<<" ";
    for(int i=0;i<3;i++)
    {
        inf>>foo>>foo>>foo>>xyz[i];
        cout<<setw(10)<<setfill(' ')<<fixed<<setprecision(3)<<xyz[i]<<" ";
    }
    cout<<endl;
}

int main()
{
    string name;
    cin>>name;
    for(int i=0; i<10000;i++)
    {
        stringstream ss;
        ss<<name<<"/"<<setw(6)<<setfill('0')<<i<<"/data";
        string path=ss.str();
        ifstream inf(path);
        if(inf)
        {
            parse(inf);
        }
        else
        {
            break;
        }
    }
}