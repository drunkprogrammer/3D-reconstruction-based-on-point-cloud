#include<iostream>
#include<fstream>
#include<vector>
#include<stdlib.h>
#include<iomanip>
using namespace std;
struct position {
	double x;
	double y;
	double z;
};
void processLine(string &line)
{
	for (char &c : line)
	{
		if (c=='\n' || c == '\t')
		{
			c = ' ';
		}
	}
}
int main()
{
	position pos;
	vector<position> data;
	char c;

	//read the data of the orange.txt
	ifstream read;
	read.open("orange.txt");

	if (!read)
	{
		cerr << "the file can not open" << endl;
	}

	while (!read.eof())
	{
		read >> pos.x >> pos.y >> pos.z;
		read >> c;
		data.push_back(pos);
	}

	read.close();

	for (int i = 0; i < data.size(); i++)
	{
		cout << setprecision(8)<<data[i].x <<" "<< data[i].y <<" "<< data[i].z <<" "<< endl;
	}
	
	system("pause");
	//return 0;
}