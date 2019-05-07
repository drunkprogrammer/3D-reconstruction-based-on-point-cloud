#include<iostream>
#include<fstream>
#include<string>
#include<vector>
#include<pcl/io/pcd_io.h>   //��д��
#include<pcl/io/ply_io.h>   //��дplyģ���ļ�
#include<pcl/point_types.h> //֧�ֵ�����
#include<pcl/filters/voxel_grid.h>
#include<pcl/filters/statistical_outlier_removal.h>
#include<pcl/kdtree/kdtree_flann.h>//kd-tree��������
#include<pcl/surface/mls.h>//��С���˷�ƽ��������
#include<pcl/visualization/pcl_visualizer.h>//���ӻ�
#include<pcl/surface/poisson.h>//�����ؽ�
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

	//���Ʋ���

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

	pcl::io::savePCDFileASCII("F:\\computer graphics\\pointCloud\\pointCloud\\pcdData\\orange.pcd", cloud);//���������ݱ�����orange.pcd��
	cerr << "Saved " << cloud.points.size() << " data points to orange.pcd" << std::endl;
}

void resample(string fileName)
{
	//��ȡ�����ļ�����

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *pointCloud) == -1)
	{
		PCL_ERROR("Cloudn't load file!");
		return;
	}

	//���Ʋ���
	pcl::VoxelGrid<pcl::PointXYZ> downSampled;//�����˲����� ͨ������ĵ������ݴ���һ����ά����դ��Ȼ��ÿ�����������е㶼�ø������е����Ľ���
	downSampled.setInputCloud(pointCloud);//������Ҫ���˵ĵ��Ƹ��˲�����
	downSampled.setLeafSize(0.06f, 0.06f, 0.06f);//�����˲�ʱ�������������Ϊ6*6*6cm3��������
	downSampled.filter(*filteredCloud);//ִ���˲������洢���cloud_filtered



	//�����˲�
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sortObject;//ͨ���Բ�ѯ��������㼯֮��ľ����ж���������Ⱥ��
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

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);//����kd-tree

	pcl::PointCloud<pcl::PointNormal> mls_points;

	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;//������С���˵Ķ���

	mls.setComputeNormals(true);//��������С���˼�������Ҫ���߹���

	//��������
	mls.setInputCloud(filteredCloud);
	mls.setPolynomialOrder(true);//����ʹ�ö���ʽ�������߾���
	mls.setSearchMethod(tree);
	mls.setSearchRadius(0.03);

	mls.process(mls_points);//������������ƽ��л�����С���˷��㷨�������ؽ�������Ľ��Ϊ����ģ�Ͳ��洢��output��

	pcl::io::savePCDFile("orange_mls", mls_points);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *pointCloud) == -1)
	{
		PCL_ERROR("Cloudn't load file!");
		return -1;
	}

	//// ���㷨����
	//pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>); //���������ƶ���ָ��
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;//���߹��ƶ���
	//pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);//�洢���Ƶķ��ߵ�ָ��
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	//tree1->setInputCloud(pointCloud);
	//n.setInputCloud(pointCloud);
	//n.setSearchMethod(tree1);
	//n.setKSearch(20);
	//n.compute(*normals); //���㷨�ߣ�����洢��normals��

	////�����ƺͷ��߷ŵ�һ��
	//pcl::concatenateFields(*pointCloud, *normals, *cloud_with_normals);


	////����poisson����,���ò���
	//pcl::Poisson<pcl::PointNormal> pn;
	//pn.setConfidence(false);//����������һ��
	//pn.setDegree(2);//ֵԽ��Խ��ϸ[1,2,3,4,5]
	//pn.setDepth(8);//�˲���
	//pn.setIsoDivide(8);//������ȡISO��ֵ����㷨���
	//pn.setManifold(false);//�Ƿ���Ӷ�������ģ�����������ǻ�ʱ����Զ���ν���ϸ�����ǻ�
	//pn.setOutputPolygons(false);//�Ƿ�������������
	//pn.setSamplesPerNode(9);//��������һ���˲�������е����������С����.
	//pn.setScale(1.25);//���������ع��������ֱ���������߽�������ֱ���ı���
	//pn.setSolverDivide(8); //����������Է������Gauss-Seidel�������������


	////����������
	//pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	//tree2->setInputCloud(cloud_with_normals);



	////���������������������
	//pn.setSearchMethod(tree2);
	//pn.setInputCloud(cloud_with_normals);
	////����������������ڴ洢���
	//pcl::PolygonMesh mesh;
	////ִ���ع�
	//pn.performReconstruction(mesh);

	////��������ͼ
	//pcl::io::savePLYFile("orange.ply", mesh);


    

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));

	viewer->setBackgroundColor(0,0,0);//��1��1��1��Ϊ��ɫ (0,0,0)Ϊ��ɫ

	//viewer->addPolygonMesh(mesh,"orange");

	viewer->addPointCloud<pcl::PointXYZ>(pointCloud,"orange cloud");

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "orange cloud");
	/*����XYZ����������Ĵ�С�ͳ��ȣ���ֵҲ����ȱʡ
	 *�鿴���ӵĵ���ͼ������û�û�з���У�Ϊ�����û�������ȷ�ķ����жϣ���Ҫ��ʾ�����ᡣ����������X��R����ɫ��
	 * Y��G����ɫ��Z��B����ɫ���ֱ������ֲ�ͬ��ɫ��Բ������档
	 */
	viewer->addCoordinateSystem(1);
	/*ͨ����������������û���Ĭ�ϵĽǶȺͷ���۲��*/
	viewer->initCameraParameters();

	/*��whileѭ�����ִ���һֱ���ڴ�״̬�����Ұ��չ涨ʱ��ˢ�´��ڡ�
	 * wasStopped()�ж���ʾ�����Ƿ��Ѿ����رգ�spinOnce()����Ϣ�ص�������������ʵ�����ø�����Ļ��ʱ��
	 */
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(1000));
	}


	cout << "finish" << endl;
	system("pause");
	return 0;
}