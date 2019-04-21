#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<pcl/io/pcd_io.h>   //读写类
#include<pcl/point_types.h> //支持点类型
using namespace std;

struct position
{
	double x;//world coordinate x
	double y;//world coordinate y
	double z;//world coordinate z
	double r;//color
};
int main()
{
	int numberTxt;
	position pos;
	vector<position> data;
	char c;

	//read the data of the orange.txt
	ifstream readTxt;
	readTxt.open("E:\\graduate\\DATA\\orange.txt");

	if (!readTxt)
	{
		cerr << "the file can not open" << endl;
	}

	while (!readTxt.eof())
	{
		readTxt >> pos.x >> pos.y >> pos.z;
		readTxt >> c;
		data.push_back(pos);
	}

	readTxt.close();

	numberTxt = data.size();

	pcl::PointCloud<pcl::PointXYZ> cloud;

	cloud.width = numberTxt;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width*cloud.height);

	for (size_t i=0;i<cloud.points.size();++i)
	{
		cloud.points[i].x = data[i].x;
		cloud.points[i].y = data[i].y;
		cloud.points[i].z = data[i].z;
	}

	pcl::io::savePCDFileASCII("E:\\graduate\\DATA\\orange.pcd",cloud);//将对象数据保存在orange.pcd中
	cerr << "Saved " << cloud.points.size() << " data points to orange.pcd" << std::endl;



	system("pause");
	return 0;
}