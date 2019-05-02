#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<pcl/io/pcd_io.h>   //读写类
#include<pcl/point_types.h> //支持点类型
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/kdtree/kdtree_flann.h>//kd-tree搜索对象
#include<pcl/surface/mls.h>//最小二乘法平滑处理类
using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

struct position
{
	double x;//world coordinate x
	double y;//world coordinate y
	double z;//world coordinate z
	double r;//color
};
void txtToPcd()
{
	int numberTxt;
	position pos;
	vector<position> data;
	char c;

	//点云采样

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

	for (size_t i = 0; i<cloud.points.size(); ++i)
	{
		cloud.points[i].x = data[i].x;
		cloud.points[i].y = data[i].y;
		cloud.points[i].z = data[i].z;
	}

	pcl::io::savePCDFileASCII("E:\\graduate\\DATA\\orange.pcd", cloud);//将对象数据保存在orange.pcd中
	cerr << "Saved " << cloud.points.size() << " data points to orange.pcd" << std::endl;
}

void resample(string fileName)
{
	//读取点云文件数据
	
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *pointCloud) == -1)
	{
		PCL_ERROR("Cloudn't load file!");
		return;
	}

	//点云采样
	pcl::VoxelGrid<pcl::PointXYZ> downSampled;
	downSampled.setInputCloud(pointCloud);
	downSampled.setLeafSize(0.1f, 0.1f, 0.1f);
	downSampled.filter(*filteredCloud);
	cout << "success" << endl;

	//点云滤波
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sortObject;//
	sortObject.setInputCloud(filteredCloud);
	sortObject.setMeanK(50);
	sortObject.setStddevMulThresh(1.0);
	sortObject.filter(*filteredCloud);
}

int main()
{
	string filename = "E:\\graduate\\DATA\\orange.pcd";
	resample(filename);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//创建kd-tree

	pcl::PointCloud<pcl::PointNormal> mls_points;

	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;//定义最小二乘的对象

	mls.setComputeNormals(true);//设置在最小二乘计算中需要法线估计

	//参数设置
	mls.setInputCloud(filteredCloud);
	mls.setPolynomialOrder(true);//设置使用多项式拟合来提高精度
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);

	mls.process(mls_points);//对所有输入点云进行基于最小二乘法算法的曲面重建，输出的结果为曲面模型并存储在output中

	pcl::io::savePCDFile("orange_mls", mls_points);

	

	cout << "finish" << endl;
	system("pause");

	return 0;
}