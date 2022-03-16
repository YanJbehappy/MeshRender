#ifndef __FILE_IO_H__
#define __FILE_IO_H__

#include <QFileInfo>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>  // loadPolygonFileOBJ
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
//#include "libLAS/header.hpp"
//#include "libLAS/factory.hpp"
//#include "libLAS/writer.hpp"
//#include "libLAS/liblas.hpp"
#include <fstream>  // std::ofstream
#include <iostream> // std::cout
#include <vector>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <map>
#include "MyCloud.h"
#include "PointCloud/pointclouds.h"



using std::vector;
using std::string;
using std::map;

class FileIO {
public:

    MyCloud load(const QFileInfo& fileInfo, std::shared_ptr<PointCloud>& pcd);
    bool save(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);

	string getInputFormatsStr() const;
	string getOutputFormatsStr() const;

	map<string, string> inputFiltersMap = {
	{"ply", "Stanford Polygon File Format (*.ply)"},
	{"pcd", "PCL Point Cloud Data (*.pcd)"},
	{"las", "las Point Cloud Data (*.las)"},
	{"xyz", "ASCII point cloud Data (*.xyz)"},
	{"txt", "ASCII point cloud Data (*.txt)"},
	{"obj", "Alias Wavefront Object (*.obj)"},
	{"stl", "STL File Format (*.stl)"},
	{"vtk", "Visualization Tookit Format (*.vtk)"}
	};

	map<string, string> outputFiltersMap = {
		{"ply", "Stanford Polygon File Format (*.ply)"},
		{"pcd", "PCL Point Cloud Data (*.pcd)"},
		{"las", "las Point Cloud Data (*.las)"},
		{"obj", "Alias Wavefront Object (*.obj)"},
		{"stl", "STL File Format (*.stl)"},
		{"vtk", "Visualization Tookit Format (*.vtk)"}
	};
private:

    std::shared_ptr<PointCloud> pcd_;
    void toPointCloudGL(const MyCloud& mycloud); // 生成OpenGL渲染需要的点云格式
    void readCloudHeader(const pcl::PCLPointCloud2& cloud2, MyCloud& mycloud); // 读取点云文件中是否含有颜色强度等信息


    MyCloud loadPLY(const QFileInfo& fileInfo);
	MyCloud loadPCD(const QFileInfo& fileInfo);
	MyCloud loadXYZ(const QFileInfo& fileInfo);
    MyCloud loadOBJ(const QFileInfo& fileInfo);
    MyCloud loadSTL(const QFileInfo& fileInfo);
    MyCloud loadVTK(const QFileInfo& fileInfo);
    MyCloud loadLAS(const QFileInfo& fileInfo);

    bool savePLY(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);
    bool savePCD(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);
    bool saveOBJ(const MyCloud& myCloud, const QFileInfo& fileInfo);
    bool saveSTL(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);
    bool saveVTK(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);
    bool saveLAS(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);


};

#endif
