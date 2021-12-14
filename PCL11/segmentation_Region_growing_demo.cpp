#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>

#include <pcl/visualization/cloud_viewer.h>

/*
* �㷨���ģ����㷨�ǻ��ڵ㷨��֮��ǶȵıȽϣ���ͼ������ƽ��Լ�������ڵ�ϲ���һ����һ�ص㼯����ʽ�����ÿ�ص㼯����Ϊ��������ͬƽ�档
* ����ԭ��������Ҫ���ף����������Ǵ�����С����ֵ(curvature value)�ĵ㿪ʼ�ġ�
* ��ˣ����Ǳ���������������ֵ���������ǽ�������������Ϊ������С�ĵ�λ��ƽ̹���򣬶�����ƽ̹�������������Լ��������������������������������������̣�
*
* 1.��������δ��ǵ㣬���յ������ֵ�Ե���������ҵ���С����ֵ�㣬��������ӵ����ӵ㼯��
*
* 2.����ÿ�����ӵ㣬�㷨���ᷢ���ܱߵ����н��ڵ㡣1������ÿ�����ڵ��뵱ǰ���ӵ�ķ��߽ǶȲ�(reg.setSmoothnessThreshold)��
*   �����ֵС�����õ���ֵ����ý��ڵ㱻�ص㿼�ǣ����еڶ������ԣ�2���ý��ڵ�ͨ���˷��߽ǶȲ���飬
*   �����������С�������趨����ֵ(reg.setCurvatureThreshold)�������ͱ���ӵ����ӵ㼯�������ڵ�ǰƽ�档
*
* 3.ͨ�����μ���ĵ㣬����ԭʼ����ȥ����
*
* 4.������С��صĵ���min��reg.setMinClusterSize���������Ϊmax��reg.setMaxClusterSize����
*
* 5.�ظ�1-3�����㷨�����ɵ�����min��max������ƽ�棬���Բ�ͬƽ���ǲ�ͬ��ɫ�������֡�
*
* 6.ֱ���㷨��ʣ��������ɵĵ�ز�������min���㷨ֹͣ������
*/ 

int main() {
	/*
	* �����ļ�
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("region_growing_tutorial.pcd", *cloud);
	std::cout << "�����ļ� size = " << cloud->size() << std::endl;

	/*
	* ��ԭʼ���ƽ��з��߹���  -- > normal
	*/
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(50);
	ne.compute(*normal);

	/*
	* �������������ָ�
	*/
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMaxClusterSize(1000000);
	reg.setMinClusterSize(50);
	reg.setSearchMethod(tree);
	//ָ������������ʱ���ӵ㸽���������ĵ���
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);
	reg.setInputNormals(normal);
	//�Ի���Ϊ��λ���ýǶȣ��ýǶȽ���������ƫ�������Χ������㷨��֮���ƫ��С��ƽ������ֵ������������ͬһ��Ⱥ��
	reg.setSmoothnessThreshold(3.5 / 180.0 * M_PI);
	//�ڶ�������������ֵ�����������ķ���ƫ���С����������ǵ�����֮��Ĳ��졣�����ֵС��������ֵ�����㷨��ʹ������ӵĵ����������Ⱥ��
	reg.setCurvatureThreshold(1.0);
	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);
	std::cout << "���������ָ�õ��ľ������ size = " << clusters.size() << std::endl;


	/*
	* ���ݿ��ӻ�
	*/
	int sum = 0;
	for (size_t i = 0; i < clusters.size(); i++) {
		std::cout << "��Ⱥ" << i + 1 << "size = " << clusters[i].indices.size() << std::endl;
		sum = sum + clusters[i].indices.size();
	}
	std::cout << "���ƹ��� size = " << cloud->size() << "���뼯Ⱥ�ĵ��� size = " << sum << "��Ч���� size = " << cloud->size() - sum << std::endl;


	/*
	* ͼ����ӻ�
	*/
	//reg.getColoredCloud()�����ط���һ������ɫ���� �����������ɫ��ͬ
	//��ɫ��ʾδ�����κ�һ����Ⱥ�ĵ�
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::PCLVisualizer viewer("v1");
	viewer.addPointCloud(colored_cloud);
	viewer.spin();
	

}
