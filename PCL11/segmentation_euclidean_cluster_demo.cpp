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
	* ��ȡ����
	*/
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	std::cout << "load data" << std::endl;
	reader.read("table_scene_lms400.pcd", *cloud);
	std::cout << "��ȡ�ĵ��ƴ�С size = " << cloud->size() << std::endl;

	/*
	* �����˲��������
	*/
	//����һ�������˲�������
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	//���崦���ĵ���
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	//�˲�������������
	vg.setInputCloud(cloud);
	//�������ش�С
	vg.setLeafSize(0.01f, 0.01f, 0.01f);
	//���й��˲���
	vg.filter(*cloud_filtered);
	std::cout << "�����˲������ĵ��ƴ�С size = " << cloud_filtered->size() << std::endl;

	/*
	* ����ƽ��ָ� ȡ�������еļ���ƽ��
	* �����ָ���������ò���
	*/
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	//�ָ����������ֵ ������ģ��
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//���÷ָ�ģ�ͺͷ�������
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	//���ò���
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);
	//���ж�ηָ� ȡ���������ƽ��
	int i = 0;
	int nr_points = (int)cloud_filtered->size();
	//ȷ������������ȡƽ�� ȷ��ʣ�µĲ����ܴﵽԭ���Ƶ�3��
	while (cloud_filtered->size() > 0.3 * nr_points) {
		//�ָ�
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		//�Ѿ��Ҳ���ƽ����
		if (inliers->indices.size() == 0) {
			std::cout << "�Ѿ���ȡ����ƽ����" << std::endl;
			break;
		}
		//����ȡ����ƽ���ԭ���ĵ�����ɾ��
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setNegative(true);
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.filter(*cloud_filtered);
		std::cout << "cloud_filtered size = " << cloud_filtered->size() << std::endl;
	}

	/*
	* �ӵ�����ȥ���˶��ƽ�� Ȼ�����ŷ����;���ָ�
	*/
	//ŷ����;���ָ����
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	//����������
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);
	//�ָ�ķ���ֵ��һ�����������ɵ�����
	std::vector< pcl::PointIndices > cluster_indices;
	//����ָ�Ĳ�������
	ec.setClusterTolerance(0.02); // 2cm
	ec.setMinClusterSize(100);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);


	/*
	* ���ӻ�
	*/
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("segmentation_euclidean_cluster"));
	int v1(0);
	int v2(1);

	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	//v1 ����
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_red(cloud, 255, 0, 0); //Ⱦɫ��
	viewer->addPointCloud(cloud_filtered, cloud_red,"cloud",v1);
	//v2 ����
	for (size_t i = 0; i < cluster_indices.size(); i++ ){
		pcl::PointCloud<pcl::PointXYZ>::Ptr c(new pcl::PointCloud<pcl::PointXYZ>);
		for (size_t j = 0; j < cluster_indices[i].indices.size(); j++) {
			c->push_back(cloud_filtered->points[cluster_indices[i].indices[j]]);
		}
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_rand(c, (c->size() * 50) % 255, (c->size() * 80) % 255, (c->size() * 120) % 255); //Ⱦɫ��
		char* x_str = new char[2];
		x_str = itoa(i, x_str, 10);
		viewer->addPointCloud(c, cloud_rand, x_str , v2);
		std::cout << "������һ������ SZIE = " << c->size() << " " << (c->size() + 1 * 50) % 255 <<" " << (c->size() + 1 * 80) % 255 <<" " << (c->size() + 1 * 120) % 255 << std::endl;
	}
	
	viewer->spin();

	 

	return 0;


}