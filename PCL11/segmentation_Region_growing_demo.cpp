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

}
