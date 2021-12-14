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
* 算法核心：该算法是基于点法线之间角度的比较，企图将满足平滑约束的相邻点合并在一起，以一簇点集的形式输出。每簇点集被认为是属于相同平面。
* 工作原理：首先需要明白，区域增长是从有最小曲率值(curvature value)的点开始的。
* 因此，我们必须计算出所有曲率值，并对它们进行排序。这是因为曲率最小的点位于平坦区域，而从最平坦的区域增长可以减少区域的总数。现在我们来具体描述这个过程：
*
* 1.点云中有未标记点，按照点的曲率值对点进行排序，找到最小曲率值点，并把它添加到种子点集；
*
* 2.对于每个种子点，算法都会发现周边的所有近邻点。1）计算每个近邻点与当前种子点的法线角度差(reg.setSmoothnessThreshold)，
*   如果差值小于设置的阈值，则该近邻点被重点考虑，进行第二步测试；2）该近邻点通过了法线角度差检验，
*   如果它的曲率小于我们设定的阈值(reg.setCurvatureThreshold)，这个点就被添加到种子点集，即属于当前平面。
*
* 3.通过两次检验的点，被从原始点云去除。
*
* 4.设置最小点簇的点数min（reg.setMinClusterSize），最大点簇为max（reg.setMaxClusterSize）。
*
* 5.重复1-3步，算法会生成点数在min和max的所有平面，并对不同平面标记不同颜色加以区分。
*
* 6.直到算法在剩余点中生成的点簇不能满足min，算法停止工作。
*/ 

int main() {
	/*
	* 载入文件
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("region_growing_tutorial.pcd", *cloud);
	std::cout << "载入文件 size = " << cloud->size() << std::endl;

	/*
	* 对原始点云进行法线估计  -- > normal
	*/
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	ne.setSearchMethod(tree);
	ne.setInputCloud(cloud);
	ne.setKSearch(50);
	ne.compute(*normal);

	/*
	* 进行区域生长分割
	*/
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMaxClusterSize(1000000);
	reg.setMinClusterSize(50);
	reg.setSearchMethod(tree);
	//指的是区域增长时种子点附近纳入检验的点数
	reg.setNumberOfNeighbours(30);
	reg.setInputCloud(cloud);
	reg.setInputNormals(normal);
	//以弧度为单位设置角度，该角度将用作法线偏差的允许范围。如果点法线之间的偏差小于平滑度阈值，则建议它们在同一集群中
	reg.setSmoothnessThreshold(3.5 / 180.0 * M_PI);
	//第二个负责曲率阈值。如果两个点的法线偏差很小，则测试它们的曲率之间的差异。如果该值小于曲率阈值，则算法将使用新添加的点继续增长集群。
	reg.setCurvatureThreshold(1.0);
	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);
	std::cout << "基于生长分割得到的聚类个数 size = " << clusters.size() << std::endl;


	/*
	* 数据可视化
	*/
	int sum = 0;
	for (size_t i = 0; i < clusters.size(); i++) {
		std::cout << "集群" << i + 1 << "size = " << clusters[i].indices.size() << std::endl;
		sum = sum + clusters[i].indices.size();
	}
	std::cout << "点云共有 size = " << cloud->size() << "加入集群的点有 size = " << sum << "无效点有 size = " << cloud->size() - sum << std::endl;


	/*
	* 图像可视化
	*/
	//reg.getColoredCloud()方法回返回一个带颜色的云 各个聚类的颜色不同
	//红色表示未加入任何一个集群的点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::PCLVisualizer viewer("v1");
	viewer.addPointCloud(colored_cloud);
	viewer.spin();
	

}
