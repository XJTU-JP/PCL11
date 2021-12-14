#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>      

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

int main_segmentation_euclidean_cluster_test1() {
//int main(){
	//读取数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("five_people.pcd", *cloud);
	std::cout << "cloud size = " << cloud->size() << std::endl;
	//std::cout << cloud->sensor_orientation_.matrix() << std::endl;

	/*
	* 预处理 
	* 先做一个体素滤波
	* 先z轴上做一个直通滤波
	* 再去x上做一个直通滤波
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filter(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid <pcl::PointXYZ> vg;
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	vg.setInputCloud(cloud);
	vg.filter(*cloud_filter);
	std::cout << "filter size = " << cloud_filter->size() << std::endl;

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setFilterFieldName("z");
	pass.setFilterLimits(1, 4);
	pass.setNegative(false);
	pass.setInputCloud(cloud_filter);
	pass.filter(*cloud_filter);
	std::cout << "filter size = " << cloud_filter->size() << std::endl;
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-1.5, 1.3);  
	pass.setNegative(false);
	pass.setInputCloud(cloud_filter);
	pass.filter(*cloud_filter);
	std::cout << "filter size = " << cloud_filter->size() << std::endl;

	

	/*
	* 做一个平面提取 .. with normal
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_five(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal_filter(new pcl::PointCloud<pcl::Normal>);
	//平面分割模块
	pcl::SACSegmentationFromNormals<pcl::PointXYZ,pcl::Normal> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//法线估计模块
	pcl::NormalEstimation<pcl::PointXYZ,pcl::Normal> en;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	en.setInputCloud(cloud_filter);
	en.setSearchMethod(tree);
	en.setKSearch(50);
	en.compute(*normal_filter);
	//根据法线进行平面分割	
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight(0.1);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.1);
	seg.setInputNormals(normal_filter);
	seg.setInputCloud(cloud_filter);
	seg.segment(*inliers, *coefficients);
	std::cout << "seg over size = " << inliers->indices.size() << std::endl;
	//去除平面
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setNegative(true);
	extract.setIndices(inliers);
	extract.setInputCloud(cloud_filter);
	extract.filter(*cloud_five);

	//聚类
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> eu;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	std::vector<pcl::PointIndices> cluster;
	tree->setInputCloud(cloud_five);
	eu.setSearchMethod(tree2);
	eu.setMaxClusterSize(25000);
	eu.setMinClusterSize(1000);
	eu.setClusterTolerance(0.025);
	eu.setInputCloud(cloud_five);
	eu.extract(cluster);


	std::cout << "cluster size = " << cluster.size() << std::endl;

	//可视化
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("segmentation_euclidean_cluster"));
	int v1(0);
	int v2(1);

	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	//v1 窗口
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_red(cloud, 255, 0, 0); //染色器
	viewer->addPointCloud(cloud_five, cloud_red, "cloud", v1);
	//v2 窗口
	for (size_t i = 0; i < cluster.size(); i++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
		for (size_t j = 0; j < cluster[i].indices.size(); j++) {
			c->push_back(cloud_five->points[cluster[i].indices[j]]);
		}
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_rand(c, (c->size() * 50) % 255, (c->size() * 80) % 255, (c->size() * 120) % 255); //染色器
		char* x_str = new char[2];
		x_str = itoa(i, x_str, 10);
		viewer->addPointCloud(c, cloud_rand, x_str, v2);
		std::cout << "加入了一个点云 SZIE = " << c->size() << " " << (c->size() + 1 * 50) % 255 << " " << (c->size() + 1 * 80) % 255 << " " << (c->size() + 1 * 120) % 255 << std::endl;
	}

	viewer->spin();

	  
	return 0;
}