#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/visualization/pcl_visualizer.h>

int main_segmentation_euclidean_cluster_demo() {
//int main(){

	/*
	* 读取数据
	*/
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::cout << "load data" << std::endl;
	reader.read("table_scene_lms400.pcd", *cloud);
	std::cout << "读取的点云大小 size = " << cloud->size() << std::endl;

	/*
	* 体素滤波处理点云
	*/
	//定义一个体素滤波处理器
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	//定义处理后的点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//滤波处理器的输入
	vg.setInputCloud(cloud);
	//定义体素大小
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	//进行过滤操作
	vg.filter(*cloud_filtered);
	std::cout << "体素滤波处理后的点云大小 size = " << cloud_filtered->size() << std::endl;

	/*
	* 先做平面分割 取出点云中的几个平面
	* 创建分割对象并且设置参数
	*/
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//分割的两个返回值 索引和模型
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//设置分割模型和方法参数
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	//设置参数
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);
	//进行多次分割 取出几个大的平面
	int i = 0;
	int nr_points = (int)cloud_filtered->size();
	//确保不是无限提取平面 确保剩下的部分能达到原点云的3成
	while (cloud_filtered->size() > 0.3 * nr_points) {
		//分割
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		//已经找不到平面了
		if (inliers->indices.size() == 0) {
			std::cout << "已经提取不到平面了" << std::endl;
			break;
		}
		//把提取到的平面从原来的点云中删除
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setNegative(true);
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.filter(*cloud_filtered);
		std::cout << "cloud_filtered size = " << cloud_filtered->size() << std::endl;
	}

	/*
	* 从点云中去除了多个平面 然后进行欧几里和聚类分割
	*/
	//欧几里和聚类分割对象
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	//设置搜索树
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);
	//分割的返回值是一个多个索引组成的数组
	std::vector< pcl::PointIndices > cluster_indices;
	//聚类分割的参数设置
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);


	/*
	* 可视化
	*/
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("segmentation_euclidean_cluster"));
	int v1(0);
	int v2(1);

	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	//v1 窗口
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_red(cloud, 255, 0, 0); //染色器
	viewer->addPointCloud(cloud_filtered, cloud_red,"cloud",v1);
	//v2 窗口
	for (size_t i = 0; i < cluster_indices.size(); i++ ){
		pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
		for (size_t j = 0; j < cluster_indices[i].indices.size(); j++) {
			c->push_back(cloud_filtered->points[cluster_indices[i].indices[j]]);
		}
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_rand(c, (c->size() * 50) % 255, (c->size() * 80) % 255, (c->size() * 120) % 255); //染色器
		char* x_str = new char[2];
		x_str = itoa(i, x_str, 10);
		viewer->addPointCloud(c, cloud_rand, x_str , v2);
		std::cout << "加入了一个点云 SZIE = " << c->size() << " " << (c->size() + 1 * 50) % 255 <<" " << (c->size() + 1 * 80) % 255 <<" " << (c->size() + 1 * 120) % 255 << std::endl;
	}
	
	viewer->spin();

	 

	return 0;


}