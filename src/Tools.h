#ifndef __TOOLS_H__
#define __TOOLS_H__

#include <string>
#include <vector>
#include <QTime>
#include <QString>

#include <PointCloud/pointclouds.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class MyCloud;

using std::string;
using std::vector;

class Tools
{
public:
	Tools();
	~Tools();



};

string getFileName(string file_name);
void timeStart();
QString timeOff();

// string to QString
QString toQString(const string& s);
// QString to string
string fromQString(const QString& qs);

string joinStrVec(const vector<string> v, string splitor = " ");

// �жϵ��Ƿ��ڶ�����ڲ�����άƽ���ϼ���
int inOrNot1n(int poly_sides, double* poly_X, double* poly_Y, double x, double y);

// ��MyCloud��������������OpenGL��Ⱦ����ĵ���
void convertToOpenGLCloud(const MyCloud& mycloud_in, PointCloud& cloud_out);

// ��pcl����ת����OpenGL��Ⱦ���͵���,��Ҫ���ⲿ��ֵname����
template <typename T>
void convertToOpenGLCloud(const pcl::PointCloud<T>& cloud_in, int r, int g, int b, PointCloud& cloud_out) {
	int points_num = cloud_in.points.size();

	//cloud_out.name = toQString(mycloud.cloudId);
	cloud_out.position.resize(3, points_num); // λ�þ���
	cloud_out.colors.resize(4, points_num); // ��ɫ����
	cloud_out.labels.resize(1, points_num); // ������Ϣ
	cloud_out.intensity.resize(1, points_num); // ǿ����Ϣ
	cloud_out.offset = Eigen::Vector3f(0.0, 0.0, 0.0); // ͷ�ļ��а�����ƫ��

#pragma omp parallel for
	for (int index = 0; index < points_num; index++) {
		// ��ȡ����
		Eigen::Vector3f point3d;
		point3d.x() = cloud_in.points[index].x;
		point3d.y() = cloud_in.points[index].y;
		point3d.z() = cloud_in.points[index].z;
		cloud_out.position.col(index) = point3d;

		// ��ȡ��ɫ
		Eigen::Vector4i_8 color;
		color.x() = r;
		color.y() = g;
		color.z() = b;
		color.w() = 255;
		cloud_out.colors.col(index) = color;

		cloud_out.labels(0, index) = (int)0;
		cloud_out.intensity(0, index) = 0;
	}

	Eigen::Vector3f minp = cloud_out.position.rowwise().minCoeff(); // ȡ��ÿһ������Сֵ �õ���С��
	Eigen::Vector3f maxp = cloud_out.position.rowwise().maxCoeff(); // ȡ��ÿһ�������ֵ �õ�����

	cloud_out.boundingBox.extend(minp);
	cloud_out.boundingBox.extend(maxp);

	cloud_out.points_num = points_num;

	// Max Min Idensity
	cloud_out.setmaxIdensity(cloud_out.intensity.rowwise().maxCoeff()(0, 0));
	cloud_out.setminIdensity(cloud_out.intensity.rowwise().minCoeff()(0, 0));

	// ���� indexTable
	cloud_out.indexTable = new uint32_t[points_num];

	cloud_out.setAttributeMode(FROM_RBG);

}
#endif
