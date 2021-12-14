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

}
