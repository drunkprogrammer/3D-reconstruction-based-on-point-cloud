#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<pcl/io/pcd_io.h>   //读写类
#include<pcl/io/ply_io.h>   //读写ply模型文件
#include<pcl/point_types.h> //支持点类型
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/kdtree/kdtree_flann.h>//kd-tree搜索对象
#include<pcl/surface/mls.h>//最小二乘法平滑处理类
#include<pcl/visualization/pcl_visualizer.h>//可视化
#include<pcl/surface/poisson.h>//泊松重建
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>

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
	cloud.points.resize(cloud.width * cloud.height);

	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		cloud.points[i].x = data[i].x;
		cloud.points[i].y = data[i].y;
		cloud.points[i].z = data[i].z;
	}

	pcl::io::savePCDFileASCII("F:\\computer graphics\\pointCloud\\pointCloud\\pcdData\\orange.pcd", cloud);//将对象数据保存在orange.pcd中
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
	pcl::VoxelGrid<pcl::PointXYZ> downSampled;//创建滤波对象 通过输入的点云数据创建一个三维体素栅格，然后每个体素内所有点都用该体素中的重心近似
	downSampled.setInputCloud(pointCloud);//设置需要过滤的点云给滤波对象
	downSampled.setLeafSize(0.06f, 0.06f, 0.06f);//设置滤波时创建的体素体积为6*6*6cm3的立方体
	downSampled.filter(*filteredCloud);//执行滤波处理，存储输出cloud_filtered



	//点云滤波
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sortObject;//通过对查询点与邻域点集之间的距离判断来过滤离群点
	sortObject.setInputCloud(filteredCloud);
	sortObject.setMeanK(50);
	sortObject.setStddevMulThresh(1.0);
	sortObject.filter(*filteredCloud);
	pcl::PCDWriter writer;
	writer.write("orange_downsampled.pcd", *filteredCloud);


	cout << "success" << endl;
}

int main()
{
	string filename = "F:\\computer graphics\\pointCloud\\pointCloud\\pcdData\\orange.pcd";

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

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *pointCloud) == -1)
	{
		PCL_ERROR("Cloudn't load file!");
		return -1;
	}

	//// 计算法向量
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //法向量点云对象指针
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//法线估计对象
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//存储估计的法线的指针
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	//tree1->setInputCloud(pointCloud);
	//n.setInputCloud(pointCloud);
	//n.setSearchMethod(tree1);
	//n.setKSearch(20);
	//n.compute(*normals); //计算法线，结果存储在normals中

	////将点云和法线放到一起
	//pcl::concatenateFields(*pointCloud, *normals, *cloud_with_normals);


	////创建poisson对象,设置参数
	//pcl::Poisson<pcl::PointNormal> pn;
	//pn.setConfidence(false);//所有向量归一化
	//pn.setDegree(2);//值越大越精细[1,2,3,4,5]
	//pn.setDepth(8);//八叉树
	//pn.setIsoDivide(8);//用于提取ISO等值面的算法深度
	//pn.setManifold(false);//是否添加多边形重心，当多边形三角化时，则对多边形进行细分三角化
	//pn.setOutputPolygons(false);//是否输出多边形网格
	//pn.setSamplesPerNode(9);//设置落入一个八叉树结点中的样本点的最小数量.
	//pn.setScale(1.25);//设置用于重构立方体的直径和样本边界立方体直径的比率
	//pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度


	////创建搜索树
	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//tree2->setInputCloud(cloud_with_normals);



	////设置搜索方法和输入点云
	//pn.setSearchMethod(tree2);
	//pn.setInputCloud(cloud_with_normals);
	////创建多变形网格，用于存储结果
	//pcl::PolygonMesh mesh;
	////执行重构
	//pn.performReconstruction(mesh);

	////保存网格图
	//pcl::io::savePLYFile("orange.ply", mesh);


    

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));

	viewer->setBackgroundColor(0,0,0);//（1，1，1）为白色 (0,0,0)为黑色

	//viewer->addPolygonMesh(mesh,"orange");

	viewer->addPointCloud<pcl::PointXYZ>(pointCloud,"orange cloud");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "orange cloud");
	/*设置XYZ三个坐标轴的大小和长度，该值也可以缺省
	 *查看复杂的点云图像会让用户没有方向感，为了让用户保持正确的方向判断，需要显示坐标轴。三个坐标轴X（R，红色）
	 * Y（G，绿色）Z（B，蓝色）分别用三种不同颜色的圆柱体代替。
	 */
	viewer->addCoordinateSystem(1);
	/*通过设置相机参数是用户从默认的角度和方向观察点*/
	viewer->initCameraParameters();

	/*此while循环保持窗口一直处于打开状态，并且按照规定时间刷新窗口。
	 * wasStopped()判断显示窗口是否已经被关闭，spinOnce()叫消息回调函数，作用其实是设置更新屏幕的时间
	 */
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}


	cout << "finish" << endl;
	system("pause");
	return 0;
}