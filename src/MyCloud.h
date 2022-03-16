#ifndef __MY_CLOUD_H__
#define __MY_CLOUD_H__

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <QFileInfo>
#include <QThread>
#include "Tools.h"

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class MyCloud
{
public:
	MyCloud();//构造时对点云和mesh进行了初始化
	~MyCloud();



	PointCloudT::Ptr cloud;      // point cloud pointer

	pcl::search::KdTree<PointT>::Ptr  KdTree_mycloud;

	pcl::PolygonMesh::Ptr mesh;  // polygon mesh pointer
	vector<float> intensity;     // points intensity 
	pcl::PointXYZ cloudScale;	 // 记录点云坐标变化尺度
	string filePath;     // dir + file name   e.g. /home/user/hello.min.ply
	string fileDir;      // only dir          e.g. /home/user
	string fileName;     // only file name    e.g. hello.min.ply
	string fileBaseName; // file name without suffx  e.g. hello
	string fileSuffix;   // file name suffx   e.g. ply

	string cloudId;      // cloud id in `viewer`: fileBaseName
	string meshId;       // mesh id in `viewer`: "mesh-" + fileName

	bool isValid = true;
	bool hasCloud = true;
	bool hasMesh = false;
	bool hasRgb = false;
	bool hasIntensity = false;

	// string filename;             // point cloud file full name
	// string subname;              // point cloud file short name
	// string dirname = "E:\\Date\\PointCloud\\";

	//bool visible;
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	string curMode;    // default show mode
	vector<string> supportedModes;


	void setPointColor(int r, int g, int b);
	void setPointColor(PointCloudT& pt_cloud);
	void setPointAlpha(int a);
	//void setShowMode(const string& mode);
	//void showCloud();
	//void hideCloud();
	//void showMesh();
	//void hideMesh();
	//void show();
	//void hide();

	void init(const QFileInfo& fileInfo, bool hasCloud, bool hasMesh);



	static MyCloud getInvalidMyCloud() {
		MyCloud myCloud;
		myCloud.isValid = false;
		return myCloud;
	}

	

};
#endif
