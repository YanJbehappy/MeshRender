#include "Tools.h"
#include "MyCloud.h"

QTime myTime;

Tools::Tools()
{
}


Tools::~Tools()
{
}

string getFileName(string file_name)
{
	string subname;
	for (auto i = file_name.end() - 1; *i != '/'; i--)
	{
		subname.insert(subname.begin(), *i);
	}
	return subname;
}

void timeStart()
{
	myTime.start();
}

QString timeOff()
{
	int timediff = myTime.elapsed();
	float f = timediff / 1000.0;
	QString tr_timediff = QString("%1").arg(f);  //float->QString
	return tr_timediff;
}

QString toQString(const string& s) {
	QString qs = QString::fromLocal8Bit(s.data());
	return qs;
}

string fromQString(const QString& qs) {
	string s = qs.toLocal8Bit();
	return s;
}

string joinStrVec(const vector<string> v, string splitor) {
	string s = "";
	if (v.size() == 0) return s;
	for (int i = 0; i != v.size()  - 1; ++i) {
		s += (v[i] + splitor);
	}
	s += v[v.size() - 1];
	return s;
}

int inOrNot1n(int poly_sides, double* poly_X, double* poly_Y, double x, double y)
{
	int i, j;
	j = poly_sides - 1;
	int res = 0;
	for (i = 0; i < poly_sides; i++)
	{
		//��ÿһ���߽��б������ñߵ������˵㣬��һ�������ڴ�����(x,y)����ߣ����������У���һ�����y��߱�p.yС����һ�����y��p.y��
		if ((poly_Y[i] < y && poly_Y[j] >= y || poly_Y[j] < y && poly_Y[i] >= y) && (poly_X[i] <= x || poly_X[j] <= x))
		{
			//��ˮƽ��ֱ����ñ��ཻ���󽻵��x���ꡣ
			res ^= ((poly_X[i] + (y - poly_Y[i]) / (poly_Y[j] - poly_Y[i]) * (poly_X[j] - poly_X[i])) < x);
		}
		j = i;
	}
	return res;
}

void convertToOpenGLCloud(const MyCloud& mycloud_in, PointCloud& cloud_out) {
	int points_num = mycloud_in.cloud->points.size();

	cloud_out.name = toQString(mycloud_in.cloudId);
	cloud_out.position.resize(3, points_num); // λ�þ���
	cloud_out.colors.resize(4, points_num); // ��ɫ����
	cloud_out.labels.resize(1, points_num); // ������Ϣ
	cloud_out.intensity.resize(1, points_num); // ǿ����Ϣ
	cloud_out.offset = Eigen::Vector3f(0.0, 0.0, 0.0); // ͷ�ļ��а�����ƫ��

	bool is8bit = false;
	// ĳЩ��ʽ�º�����ɫ��Ϣ
	if (mycloud_in.hasRgb) {
		if (mycloud_in.cloud->points[0].a <= 255)
			is8bit = true;
	}
	// Read Point
#pragma omp parallel for
	for (int index = 0; index < points_num; index++) {
		// ��ȡ����
		Eigen::Vector3f point3d;
		point3d.x() = mycloud_in.cloud->points[index].x;
		point3d.y() = mycloud_in.cloud->points[index].y;
		point3d.z() = mycloud_in.cloud->points[index].z;
		cloud_out.position.col(index) = point3d;

		// ��ȡ��ɫ
		Eigen::Vector4i_8 color;
		if (mycloud_in.hasRgb) {
			color.x() = is8bit ? mycloud_in.cloud->points[index].r : mycloud_in.cloud->points[index].r / 257;
			color.y() = is8bit ? mycloud_in.cloud->points[index].g : mycloud_in.cloud->points[index].g / 257;
			color.z() = is8bit ? mycloud_in.cloud->points[index].b : mycloud_in.cloud->points[index].b / 257;
			color.w() = is8bit ? mycloud_in.cloud->points[index].a : mycloud_in.cloud->points[index].a / 257;
		}
		// ������ɫ��Ϣ�򸳰�ģ
		else {
			color.x() = 255;
			color.y() = 255;
			color.z() = 255;
			color.w() = 0;
		}
		cloud_out.colors.col(index) = color;

		cloud_out.labels(0, index) = (int)0;
		cloud_out.intensity(0, index) = (int)mycloud_in.intensity[index];
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
}