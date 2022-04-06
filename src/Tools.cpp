#include "Tools.h"
#include "MyCloud.h"
#include "Base/tool.h"

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
	for (int i = 0; i != v.size() - 1; ++i) {
		s += (v[i] + splitor);
	}
	s += v[v.size() - 1];
	return s;
}

// 获取点云转换矩阵
Eigen::Matrix4f getModelMatrixToOrigin(Eigen::Vector3f origin) {
	// 点云编辑平移值
	Eigen::Vector3f translate = Eigen::Vector3f::Identity();
	// 点云编辑旋转值
	Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
	// 点云编辑尺度缩放
	float zoom = 1.0;
	// 自动平移至中心位置
	Eigen::Matrix4f transTocenter = Eigen::Affine3f(Eigen::Translation<float, 3>(-origin)).matrix();
	// 用户定义平移量
	Eigen::Matrix4f trans = Eigen::Affine3f(Eigen::Translation<float, 3>(translate)).matrix();
	//先旋转 后平移
	Eigen::Matrix4f model = trans * rotation * Eigen::Affine3f(Eigen::Scaling(Eigen::Vector3f::Constant(zoom))).matrix() * transTocenter;
	return model;
}

int inOrNot1n(int poly_sides, double* poly_X, double* poly_Y, double x, double y)
{
	int i, j;
	j = poly_sides - 1;
	int res = 0;
	for (i = 0; i < poly_sides; i++)
	{
		//对每一条边进行遍历，该边的两个端点，有一个必须在待检测点(x,y)的左边，且两个点中，有一个点的y左边比p.y小，另一个点的y比p.y大。
		if ((poly_Y[i] < y && poly_Y[j] >= y || poly_Y[j] < y && poly_Y[i] >= y) && (poly_X[i] <= x || poly_X[j] <= x))
		{
			//用水平的直线与该边相交，求交点的x坐标。
			res ^= ((poly_X[i] + (y - poly_Y[i]) / (poly_Y[j] - poly_Y[i]) * (poly_X[j] - poly_X[i])) < x);
		}
		j = i;
	}
	return res;
}

void convertToOpenGLCloud(const MyCloud& mycloud_in, PointCloud& cloud_out) {
	int points_num = mycloud_in.cloud->points.size();

	cloud_out.name = toQString(mycloud_in.cloudId);
	cloud_out.position.resize(3, points_num); // 位置矩阵
	cloud_out.colors.resize(4, points_num); // 颜色矩阵
	cloud_out.labels.resize(1, points_num); // 分类信息
	cloud_out.intensity.resize(1, points_num); // 强度信息
	cloud_out.offset = Eigen::Vector3f(0.0, 0.0, 0.0); // 头文件中包含的偏移

	bool is8bit = false;
	// 某些格式下含有颜色信息
	if (mycloud_in.hasRgb) {
		if (mycloud_in.cloud->points[0].a <= 255)
			is8bit = true;
	}
	// Read Point
#pragma omp parallel for
	for (int index = 0; index < points_num; index++) {
		// 读取坐标
		Eigen::Vector3f point3d;
		point3d.x() = mycloud_in.cloud->points[index].x;
		point3d.y() = mycloud_in.cloud->points[index].y;
		point3d.z() = mycloud_in.cloud->points[index].z;
		cloud_out.position.col(index) = point3d;

		// 读取颜色
		Eigen::Vector4i_8 color;
		if (mycloud_in.hasRgb) {
			color.x() = is8bit ? mycloud_in.cloud->points[index].r : mycloud_in.cloud->points[index].r / 257;
			color.y() = is8bit ? mycloud_in.cloud->points[index].g : mycloud_in.cloud->points[index].g / 257;
			color.z() = is8bit ? mycloud_in.cloud->points[index].b : mycloud_in.cloud->points[index].b / 257;
			color.w() = is8bit ? mycloud_in.cloud->points[index].a : mycloud_in.cloud->points[index].a / 257;
		}
		// 不含颜色信息则赋白模
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

	Eigen::Vector3f minp = cloud_out.position.rowwise().minCoeff(); // 取出每一列中最小值 得到最小点
	Eigen::Vector3f maxp = cloud_out.position.rowwise().maxCoeff(); // 取出每一列中最大值 得到最大点

	cloud_out.boundingBox.extend(minp);
	cloud_out.boundingBox.extend(maxp);

	cloud_out.points_num = points_num;

	// Max Min Idensity
	cloud_out.setmaxIdensity(cloud_out.intensity.rowwise().maxCoeff()(0, 0));
	cloud_out.setminIdensity(cloud_out.intensity.rowwise().minCoeff()(0, 0));

	// 分配 indexTable
	cloud_out.indexTable = new uint32_t[points_num];
}

vector<Point> Bresenham_Circle(const Point& center, const int& radius)
{
	int x, y, d;
	x = 0;
	y = radius;
	d = 3 - 2 * radius;

	// 根据圆的对称性，利用八分之一圆弧的点生成另外的点
	vector<Point> arc_1, arc_2, arc_3, arc_4, arc_5, arc_6, arc_7, arc_8;
	{// 添加第一个点，这里没有添加45°、90°、135°、180°、225°、270°、315°对应点
		Point point_1(x, y), point_8(-x, y);
		point_1 += center, point_8 += center;
		arc_1.emplace_back(point_1), arc_8.emplace_back(point_8);
	}
	while (x < y)
	{
		if (d < 0)
		{
			d = d + 4 * x + 6;
		}
		else
		{
			d = d + 4 * (x - y) + 10;
			y--;
		}
		x++;

		Point point_1(x, y), point_2(y, x), point_3(y, -x), point_4(x, -y), point_5(-x, -y), point_6(-y, -x), point_7(-y, x), point_8(-x, y);
		point_1 += center, point_2 += center, point_3 += center, point_4 += center, point_5 += center, point_6 += center, point_7 += center, point_8 += center;
		arc_1.emplace_back(point_1), arc_2.emplace_back(point_2), arc_3.emplace_back(point_3), arc_4.emplace_back(point_4);
		arc_5.emplace_back(point_5), arc_6.emplace_back(point_6), arc_7.emplace_back(point_7), arc_8.emplace_back(point_8);
	}
	// 将圆上点按顺时针排列添加到一个数组中
	// 0~45度区间为arc_1, 45~90度区间为arc_2
	for (int i = arc_2.size() - 1; i >= 0; i--)
	{
		arc_1.push_back(arc_2[i]);
	}
	// 90~135度区间
	arc_1.insert(arc_1.end(), arc_3.begin(), arc_3.end());
	// 135~180度区间
	for (int i = arc_4.size() - 1; i >= 0; i--)
	{
		arc_1.push_back(arc_4[i]);
	}
	// 180~225度区间
	arc_1.insert(arc_1.end(), arc_5.begin(), arc_5.end());
	// 225~270度区间
	for (int i = arc_6.size() - 1; i >= 0; i--)
	{
		arc_1.push_back(arc_6[i]);
	}
	// 270~315度区间
	arc_1.insert(arc_1.end(), arc_7.begin(), arc_7.end());
	// 315~360度区间
	for (int i = arc_8.size() - 1; i >= 0; i--)
	{
		arc_1.push_back(arc_8[i]);
	}

	return arc_1;
}